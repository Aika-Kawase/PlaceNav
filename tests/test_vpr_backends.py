"""Run inside the ROS container: python3 -m unittest discover -s tests -p test_vpr_backends.py"""
import json
from pathlib import Path
import tempfile
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

import h5py
import numpy as np
import torch
from torch import nn

from placenav.models.pr_models.torchscript_vpr import TorchScriptVPR
from placenav.place_recognition.vpr_config import image_transform, database_signature
from placenav.place_recognition import extract_database
from placenav.place_recognition.bayesian_querier import PlaceRecognitionTopologicalFilter


class TinyVPR(nn.Module):
    def forward(self, image):
        return torch.nn.functional.normalize(image.mean(dim=(2, 3)), dim=1)


class VPRTests(unittest.TestCase):
    def test_gnm_and_vpr_receive_independent_sizes(self):
        source = np.full((186, 288, 3), 127, dtype=np.uint8)
        self.assertEqual(tuple(image_transform([85, 64])(source).shape), (3, 64, 85))
        self.assertEqual(tuple(image_transform([322, 322], True)(source).shape), (3, 322, 322))

    def test_offline_adapter_and_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'tiny.pt'
            metadata = dict(backend='test', descriptor_dim=3, input_size=[14, 14])
            torch.jit.trace(TinyVPR(), torch.randn(1, 3, 14, 14)).save(
                str(path), _extra_files={'metadata.json': json.dumps(metadata)})
            adapter = TorchScriptVPR(dict(checkpoint_path=path, **metadata))
            output = adapter({'image': torch.randn(1, 3, 14, 14)})['global_descriptor']
            self.assertEqual(tuple(output.shape), (1, 3))
            torch.testing.assert_close(output.norm(dim=1), torch.ones(1))
            with self.assertRaises(ValueError):
                adapter({'image': torch.randn(1, 3, 28, 28)})
            with self.assertRaises(ValueError):
                TorchScriptVPR(dict(checkpoint_path=path, **{**metadata, 'backend': 'wrong'}))

    def test_signature_detects_weights_image_and_preprocessing_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            weights = root / 'model'
            weights.write_bytes(b'weights')
            (root / '0.jpg').write_bytes(b'image')
            conf = dict(model=dict(checkpoint_path=weights), preprocessing=dict(resize=[14, 14]))
            first = database_signature(conf, root, ['0.jpg'])
            (root / '0.jpg').write_bytes(b'new image')
            second = database_signature(conf, root, ['0.jpg'])
            weights.write_bytes(b'new weights')
            third = database_signature(conf, root, ['0.jpg'])
            conf['preprocessing']['resize'] = [28, 28]
            fourth = database_signature(conf, root, ['0.jpg'])
            self.assertEqual(len({first, second, third, fourth}), 4)

    def test_db_rejects_partial_or_wrong_signature(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'features.h5'
            with h5py.File(path, 'w') as fd:
                fd.create_dataset('0.jpg/global_descriptor', data=[1., 0.])
                fd.attrs['vpr_signature'] = 'a'
            self.assertTrue(extract_database.database_matches_images(path, ['0.jpg'], 'a'))
            self.assertFalse(extract_database.database_matches_images(path, ['0.jpg'], 'b'))
            self.assertFalse(extract_database.database_matches_images(path, ['0.jpg', '1.jpg'], 'a'))

    def test_vlad_matches_explicit_residual_sums(self):
        from prepare_vpr import HardVLAD
        torch.manual_seed(42)
        centers = torch.randn(4, 6)
        features = torch.nn.functional.normalize(torch.randn(1, 20, 6), dim=-1)
        labels = (features[0] @ torch.nn.functional.normalize(centers, dim=-1).T).argmax(-1)
        residuals = torch.stack([(features[0, labels == k] - centers[k]).sum(0) for k in range(4)])
        expected = torch.nn.functional.normalize(
            torch.nn.functional.normalize(residuals, dim=-1).flatten(), dim=0)
        torch.testing.assert_close(HardVLAD(centers)(features)[0], expected)

    @unittest.skipUnless(torch.cuda.is_available(), 'CUDA required')
    def test_traced_allocations_move_to_gpu(self):
        from placenav.models.pr_models.script_device import set_graph_device
        class Allocate(nn.Module):
            def forward(self, image):
                out = torch.ones(image.shape, device=image.device)
                return out + image
        model = torch.jit.trace(Allocate(), torch.zeros(1, 3)).cuda()
        model = set_graph_device(model, torch.device('cuda:0'))
        torch.testing.assert_close(model(torch.ones(1, 3, device='cuda')), torch.full((1, 3), 2., device='cuda'))

    def test_bayesian_roundoff_and_identical_descriptors_are_finite(self):
        with patch('placenav.place_recognition.bayesian_querier.PlaceRecognitionDBHandler') as handler:
            handler.return_value.get_descriptors.return_value = np.array([[1.000001, 0.]] * 3)
            filt = PlaceRecognitionTopologicalFilter(
                lambda _: {'global_descriptor': torch.tensor([[1., 0.]])}, 'unused', 'unused')
            filt.initialize_model(None)
            self.assertTrue(np.isfinite(filt.belief).all())
            self.assertTrue(np.isfinite(filt.obs_lhood(np.array([1., 0.]))).all())


if __name__ == '__main__':
    unittest.main()
