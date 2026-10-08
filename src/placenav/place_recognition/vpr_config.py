"""ROS-independent preprocessing and feature database signatures."""
import hashlib
import json

from torchvision import transforms


def image_transform(size, normalize_first=False):
    resize = transforms.Resize(list(size)[::-1], antialias=True)
    normalize = transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    steps = [normalize, resize] if normalize_first else [resize, normalize]
    return transforms.Compose([transforms.ToTensor()] + steps)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def database_signature(conf, image_dir, names):
    # Hash contents, not absolute paths: moving the map to the robot preserves validity.
    value = {
        'version': 1,
        'model': {k: v for k, v in conf['model'].items() if k != 'checkpoint_path'},
        'preprocessing': conf['preprocessing'],
        'checkpoint': sha256_file(conf['model']['checkpoint_path']),
        'images': [(name, sha256_file(image_dir / name)) for name in names],
    }
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
