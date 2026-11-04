#!/usr/bin/env python3
"""Read one recorded observation using the existing CPU dataset environment."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--episode', type=int, required=True)
    parser.add_argument('--frame', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import numpy as np
    from lerobot.datasets import LeRobotDataset
    dataset = LeRobotDataset(repo_id='local/rad_place_merged_v5',
                             root=str(args.dataset), episodes=[args.episode],
                             video_backend='pyav', return_uint8=False)
    if args.frame >= len(dataset):
        raise IndexError(f'Frame {args.frame} is outside the selected episode ({len(dataset)} frames).')
    sample = dataset[args.frame]
    fields = ['observation.state', 'observation.images.global', 'observation.images.wrist']
    payload = {key: sample[key].detach().cpu().numpy() for key in fields}
    payload['task'] = np.asarray(sample['task'])
    payload['recorded_action'] = sample['action'].detach().cpu().numpy()
    payload['episode_index'] = np.asarray(args.episode)
    payload['frame_index'] = np.asarray(args.frame)
    np.savez_compressed(args.output, **payload)
    print('Exported recorded frame:', args.output, flush=True)


if __name__ == '__main__':
    main()
