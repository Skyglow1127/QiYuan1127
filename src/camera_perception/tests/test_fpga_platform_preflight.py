import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1]/'fpga/grasp_preprocess_v1/platform_preflight.py'
SPEC = importlib.util.spec_from_file_location('platform_preflight', SCRIPT)
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


class PlatformPreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.put('/proc/sys/kernel/random/boot_id', 'fixture-boot')
        self.put('/sys/kernel/config/device-tree/overlays/grasp-preprocess/status', 'applied\n')
        self.put('/sys/class/fpga_manager/fpga0/state', 'operating\n')
        self.clock_summary()
        self.put('/sys/kernel/debug/devices_deferred', '')
        self.device('grasp-preprocess-clock', 'xlnx,fclk', 'xilinx_fclk')
        self.device('grasp-preprocess-afi', 'xlnx,afi-fpga', 'afi-fpga')
        node = '/sys/firmware/devicetree/base/amba/grasp-preprocess-clock/'
        self.put(node+'clocks', struct.pack('>II', 7, 71))
        self.put(node+'assigned-clocks', struct.pack('>II', 7, 71))
        self.put(node+'assigned-clock-rates', struct.pack('>I', 100_000_000))

    def put(self, name, data):
        p = self.root/name.lstrip('/')
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data if isinstance(data, bytes) else data.encode())
        return p

    def device(self, name, compatible, driver):
        node = self.put('/sys/firmware/devicetree/base/amba/'+name+'/compatible', compatible+'\0').parent
        d = self.root/'sys/bus/platform/devices'/name
        d.mkdir(parents=True, exist_ok=True)
        (d/'of_node').symlink_to(node)
        driver_path = self.root/'sys/bus/platform/drivers'/driver
        driver_path.mkdir(parents=True, exist_ok=True)
        (d/'driver').symlink_to(driver_path)

    def clock_summary(self, enable=1, prepare=1, rate=99_999_999, protect=True):
        header = ' enable prepare '+('protect ' if protect else '')+'rate\n clock count count '+('count ' if protect else '')+'rate\n'
        row = f' pl0_ref {enable} {prepare} '+('0 ' if protect else '')+f'{rate} 0 0 50000 Y\n'
        self.put('/sys/kernel/debug/clk/clk_summary', header+row)

    def inspect(self):
        return preflight.inspect(self.root)

    def codes(self, result, kind='failed'):
        return {item['code'] for item in result[kind]}

    def test_complete_metadata_still_requires_runtime_reset_evidence(self):
        result = self.inspect()
        self.assertTrue(result['metadata_checks_passed'])
        self.assertFalse(result['ready'])
        self.assertFalse(result['hardware_access_authorized'])
        self.assertFalse(result['mmio_performed'])
        self.assertFalse(result['root_cause_resolved'])
        self.assertIn('not permanent permission', result['ready_scope'])
        self.assertEqual(self.codes(result, 'missing'), {'reset_deassertion_unverified'})

    def test_operating_uio_and_registered_drivers_are_insufficient(self):
        for name in ('grasp-preprocess-clock', 'grasp-preprocess-afi'):
            (self.root/'sys/bus/platform/devices'/name/'driver').unlink()
        self.put('/sys/class/uio/uio4/maps/map0/addr', '0xa0020000')
        result = self.inspect()
        self.assertIn('driver_not_bound', self.codes(result))
        self.assertFalse(result['metadata_checks_passed'])
        self.assertFalse(result['ready'])

    def test_starter_overlay_is_rejected(self):
        (self.root/'sys/kernel/config/device-tree/overlays/grasp-preprocess').rename(
            self.root/'sys/kernel/config/device-tree/overlays/k26-starter-kits_image_1')
        self.assertIn('unexpected_overlay_set', self.codes(self.inspect()))

    def test_disabled_or_unprepared_clock_rejected(self):
        for enable, prepare in ((0, 1), (1, 0), (0, 0)):
            with self.subTest(enable=enable, prepare=prepare):
                self.clock_summary(enable, prepare)
                self.assertIn('pl0_clock_not_enabled_and_prepared', self.codes(self.inspect()))

    def test_clock_rate_and_legacy_summary(self):
        self.clock_summary(protect=False)
        self.assertTrue(self.inspect()['metadata_checks_passed'])
        self.clock_summary(rate=96_974_000)
        self.assertIn('pl0_rate_out_of_range', self.codes(self.inspect()))

    def test_unreadable_clock_summary_fails_closed(self):
        original = preflight.MetadataFS.read
        def restricted(fs, name, binary=False):
            if name.endswith('/clk_summary'):
                raise PermissionError('root permission required')
            return original(fs, name, binary)
        with mock.patch.object(preflight.MetadataFS, 'read', restricted):
            result = self.inspect()
        self.assertIn('clk_summary_unavailable', self.codes(result, 'missing'))
        self.assertFalse(result['metadata_checks_passed'])

    def test_missing_and_relevant_deferred_probe(self):
        p = self.root/'sys/kernel/debug/devices_deferred'
        p.unlink()
        self.assertIn('deferred_probes_unavailable', self.codes(self.inspect(), 'missing'))
        self.put('/sys/kernel/debug/devices_deferred', 'amba:grasp-preprocess-clock\t supplier unavailable\n')
        self.assertIn('relevant_probe_deferred', self.codes(self.inspect()))
        self.put('/sys/kernel/debug/devices_deferred', 'unrelated-audio\t supplier unavailable\n')
        self.assertTrue(self.inspect()['metadata_checks_passed'])

    def test_mismatched_dt_clock_declarations_rejected(self):
        self.put('/sys/firmware/devicetree/base/amba/grasp-preprocess-clock/clocks', struct.pack('>II', 7, 72))
        codes = self.codes(self.inspect())
        self.assertIn('unexpected_clock_reference', codes)
        self.assertIn('assigned_clock_mismatch', codes)

    def test_malformed_or_ambiguous_clock_summary_fails_closed(self):
        for contents in ('unknown header\npl0_ref 1 1 100000000\n',
                         'enable prepare rate\npl0_ref 1 1 100000000\npl0_ref 1 1 100000000\n'):
            self.put('/sys/kernel/debug/clk/clk_summary', contents)
            self.assertIn('pl0_clock_unparseable', self.codes(self.inspect(), 'missing'))

    def test_never_reads_hardware_status_or_device_nodes(self):
        original = Path.read_bytes
        touched = []
        def tracked(path):
            touched.append(str(path))
            if '/device/status' in str(path) or str(path).startswith(('/dev/', '/sys/bus/pci')):
                raise AssertionError('Forbidden hardware access')
            return original(path)
        with mock.patch.object(Path, 'read_bytes', tracked):
            result = self.inspect()
        self.assertTrue(touched)
        self.assertFalse(any('/device/status' in p for p in result['read_paths']))
        self.assertFalse(any('/dev/' in p for p in result['read_paths']))

    def test_metadata_symlink_cannot_escape_to_device(self):
        p = self.root/'sys/kernel/debug/clk/clk_summary'
        p.unlink()
        p.symlink_to('/dev/mem')
        self.assertIn('clk_summary_unavailable', self.codes(self.inspect(), 'missing'))

    def test_cli_is_readonly_and_returns_diagnostic_failure_status(self):
        result = subprocess.run([sys.executable, str(SCRIPT), '--root', str(self.root)],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 2)
        data = json.loads(result.stdout)
        self.assertTrue(data['metadata_checks_passed'])
        self.assertFalse(data['ready'])


if __name__ == '__main__':
    unittest.main()
