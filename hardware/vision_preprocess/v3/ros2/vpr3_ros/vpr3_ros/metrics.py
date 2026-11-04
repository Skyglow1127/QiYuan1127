"""Small timing helpers; wall-clock durations are reported without inference."""
import math


def server_stage_distributions(records):
    """Summarize duration fields; leave byte counts and hardware status raw.

    The service's historical timing_ns payload also includes DMA diagnostics.
    Only *_ns fields and these documented service durations have time units.
    """
    service_durations = {
        'camera_fetch', 'jpeg_decode_and_rgb', 'preprocess_wall',
        'output_serialize', 'integrity_hash', 'server_total', 'server_cpu_time',
    }
    keys = sorted({key for record in records for key, value in record['timing_ns'].items()
                   if (key.endswith('_ns') or key in service_durations)
                   and isinstance(value, (int, float)) and not isinstance(value, bool)})
    return {key: distribution([record['timing_ns'][key] for record in records
                               if key in record['timing_ns']]) for key in keys}


def distribution(values, scale=1e-6):
    ordered = sorted(values)
    if not ordered:
        return {'n': 0, 'p50_ms': None, 'p95_ms': None}
    def percentile(fraction):
        position = (len(ordered)-1)*fraction
        lo, hi = math.floor(position), math.ceil(position)
        return (ordered[lo]+(ordered[hi]-ordered[lo])*(position-lo))*scale
    return {'n': len(ordered), 'p50_ms': percentile(.5), 'p95_ms': percentile(.95)}


def observed_rate(timestamps):
    if len(timestamps) < 2 or timestamps[-1] <= timestamps[0]:
        return None
    return (len(timestamps)-1)*1e9/(timestamps[-1]-timestamps[0])
