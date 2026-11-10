"""Summarize a small visual-review fixture, explicitly not an accuracy benchmark."""
import argparse
import json
from pathlib import Path
import statistics

from PIL import Image
from grasp_semantics import parse_objects, box_overlap, pixel_box


def evaluate(path):
    results = json.loads(Path(path).read_text())
    summary = {'results_file':str(Path(path).resolve()), 'cases':len(results), 'positive_cases':0,
               'negative_cases':0, 'present_returned':0, 'missed_present':0,
               'absent_rejected':0, 'false_present':0, 'malformed_or_incomplete':0,
               'positive_box_iou_ge_0_5':0, 'details':[]}
    times = []
    for row in results:
        case = row['case']; expected = case['expected_present']
        summary['positive_cases' if expected else 'negative_cases'] += 1
        detail = {'case':case['id'], 'expected_present':expected}
        timing = row.get('timing', {})
        if timing.get('grounding_ms') is not None: times.append(timing['grounding_ms'])
        try:
            if 'error' in row or timing.get('generation_hit_token_limit') or timing.get('generation_hit_time_limit'):
                raise ValueError('Inference failed or output bound reached')
            objects = parse_objects(row['raw_response'], max_objects=1)
            detail['returned_present'] = bool(objects)
            if expected:
                summary['present_returned' if objects else 'missed_present'] += 1
                if objects:
                    width,height = Image.open(case['image']).size
                    box = pixel_box(objects[0]['bbox_2d'],width,height)
                    overlap = box_overlap(box,case['review_box_xyxy_pixels'])
                    detail['box_iou_vs_approximate_review'] = overlap
                    if overlap>=.5: summary['positive_box_iou_ge_0_5'] += 1
            else:
                summary['false_present' if objects else 'absent_rejected'] += 1
        except (KeyError, ValueError, json.JSONDecodeError) as error:
            summary['malformed_or_incomplete'] += 1; detail['error'] = str(error)
        summary['details'].append(detail)
    if times:
        summary['median_grounding_ms'] = statistics.median(times)
        summary['max_grounding_ms'] = max(times)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('results',type=Path,nargs='+')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    report={'scope':'small local regression; 4 original scenes plus 2 empty crops; assistant approximate visual references, not human annotated AP or grasp success',
            'results':[evaluate(path) for path in a.results]}
    a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    for row in report['results']:print(json.dumps({k:v for k,v in row.items() if k!='details'},ensure_ascii=False))


if __name__=='__main__':main()
