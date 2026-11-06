#!/usr/bin/env python3
"""Independent offline VPR and GNM inference checks; publishes no ROS commands."""
import argparse
from pathlib import Path
import sys
import time
import shutil
import tempfile

import numpy as np
import torch
import yaml

# Legacy pickled GNM checkpoints refer to the top-level `models` package.
sys.path.insert(0, str(Path(__file__).resolve().parent / 'placenav'))
from placenav.models.pr_models.base_model import dynamic_load
from placenav.models import pr_models
from placenav.place_recognition.vpr_config import image_transform, database_signature
from placenav.place_recognition import extract_database
from placenav.place_recognition.bayesian_querier import PlaceRecognitionTopologicalFilter
from placenav.utils import load_model, read_image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models', nargs='+', default=['cosplace', 'megaloc', 'salad', 'anyloc'])
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    parser.add_argument('--check-db', action='store_true', help='Also build a temporary two-image DB and query it')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    confs = yaml.safe_load((root / 'config/models.yaml').read_text())
    rgb = read_image(args.image).astype(np.uint8)
    for name in args.models:
        conf = confs[name]
        model_conf = dict(conf['model'])
        model_conf['checkpoint_path'] = root / 'model_weights' / model_conf['checkpoint_path']
        model = dynamic_load(pr_models, model_conf['name'])(model_conf).eval().to(args.device)
        prep = conf.get('preprocessing', {'resize': [85, 64]})
        image = image_transform(prep['resize'], prep.get('normalize_first', False))(rgb)[None].to(args.device)
        with torch.inference_mode():
            first = model({'image': image})['global_descriptor']
            for _ in range(4):
                model({'image': image})
            if args.device == 'cuda':
                torch.cuda.synchronize()
            start = time.perf_counter()
            second = model({'image': image})['global_descriptor']
            if args.device == 'cuda':
                torch.cuda.synchronize()
            elapsed = time.perf_counter() - start
        torch.testing.assert_close(first, second)
        torch.testing.assert_close(first.norm(dim=-1), torch.ones(1, device=args.device))
        print(f'{name}: input={tuple(image.shape)}, descriptor={tuple(first.shape)}, warm={elapsed*1000:.1f} ms')
        del model, image, first, second
        if args.device == 'cuda':
            torch.cuda.empty_cache()
        if args.check_db:
            with tempfile.TemporaryDirectory(prefix='placenav-vpr-') as directory:
                import h5py
                folder = Path(directory)
                for filename in ('0.jpg', '1.jpg'):
                    shutil.copyfile(args.image, folder / filename)
                db_conf = dict(conf, model=model_conf, preprocessing=prep)
                signature = database_signature(db_conf, folder, ['0.jpg', '1.jpg'])
                transform = image_transform(prep['resize'], prep.get('normalize_first', False))
                db_path = extract_database.main(db_conf, folder, transform, folder,
                                                as_half=False, signature=signature,
                                                image_list=['0.jpg', '1.jpg'])
                assert extract_database.database_matches_images(db_path, ['0.jpg', '1.jpg'], signature)
                with h5py.File(db_path, 'r') as db:
                    desc = torch.from_numpy(db['0.jpg/global_descriptor'][:])[None]
                filt = PlaceRecognitionTopologicalFilter(lambda _: {'global_descriptor': desc}, db_path, folder)
                filt.initialize_model(None)
                node, score = filt.match(None)
                assert node in (0, 1) and np.isfinite(score)
                print(f'{name}: DB extraction/reuse and Bayesian query OK')
    gnm_conf = confs['gnm_large']
    gnm = load_model(root / 'model_weights/gnm_large.pth', gnm_conf, torch.device(args.device)).eval()
    gnm_image = image_transform([85, 64])(rgb)[None].to(args.device)
    with torch.inference_mode():
        distance, path = gnm(gnm_image.repeat(1, 6, 1, 1), gnm_image)
    assert path.shape[1] == 5 and torch.isfinite(path).all()
    print(f'GNM: input={tuple(gnm_image.shape)}, path={tuple(path.shape)}, finite=True')


if __name__ == '__main__':
    main()
