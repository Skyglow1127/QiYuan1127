#!/usr/bin/python3
"""Run the existing stationary test with durable MMIO progress markers."""
import os
from pathlib import Path
from sw import board_desc_smoke as smoke


class TracedMMIO(smoke.NativeMMIO):
    def __init__(self, build, out, platform_sha, device_mapping):
        self.trace = os.open(out/'mmio_progress.log', os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_DSYNC, 0o644)
        self.detailed = True
        self.seen = set()
        os.sync()  # Persist directory entries before the first hardware access.
        self.mark('native_backend_init begin')
        super().__init__(build, out, platform_sha, device_mapping)
        self.mark('native_backend_init done')

    def mark(self, message):
        os.write(self.trace, (message+'\n').encode())
        print(message, flush=True)

    def read32(self, region, offset):
        tracked = self.detailed or ('read',region) not in self.seen
        self.seen.add(('read',region))
        label = f'read {region}+0x{offset:03x}'
        if tracked:self.mark(label+' begin')
        value = super().read32(region, offset)
        if tracked:self.mark(label+f' done 0x{value:08x}')
        return value

    def write32(self, region, offset, value):
        tracked = self.detailed or (region=='cfg' and offset==16) or ('write',region) not in self.seen
        self.seen.add(('write',region))
        label = f'write {region}+0x{offset:03x} = 0x{value:08x}'
        if region=='cfg' and offset==16 and value==2:
            diagnostic = {f'0x{a:03x}': super(TracedMMIO,self).read32('cfg',a) for a in (12,20,24,28,32,36,40,44,0x90,0x94,0x400,0x404)}
            self.mark('ARM preflight '+str(diagnostic))
        if tracked:self.mark(label+' begin')
        super().write32(region, offset, value)
        if tracked:self.mark(label+' done')
        if region=='cfg' and offset==16 and value==1:self.detailed=False

    def close(self):
        super().close()
        if self.trace is not None:
            self.mark('native_backend_close')
            os.close(self.trace)
            self.trace = None


if __name__ == '__main__':
    smoke.NativeMMIO = TracedMMIO
    raise SystemExit(smoke.main())
