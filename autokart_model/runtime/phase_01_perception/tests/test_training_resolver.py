import pytest
from autokart_perception.training import _image_and_label_dirs


def test_split_first_layout(tmp_path):
    (tmp_path / 'train/images').mkdir(parents=True)
    (tmp_path / 'train/labels').mkdir()
    img, lbl = _image_and_label_dirs({'train': 'train/images'}, 'train', tmp_path)
    assert img == (tmp_path / 'train/images').resolve()
    assert lbl == (tmp_path / 'train/labels').resolve()


def test_images_first_layout(tmp_path):
    (tmp_path / 'images/train').mkdir(parents=True)
    (tmp_path / 'labels/train').mkdir(parents=True)
    img, lbl = _image_and_label_dirs({'train': 'images/train'}, 'train', tmp_path)
    assert img == (tmp_path / 'images/train').resolve()
    assert lbl == (tmp_path / 'labels/train').resolve()


@pytest.mark.parametrize('bad', ['../train/images', '/abs/train/images', 'train/other', 'images'])
def test_rejects_unsafe_or_unknown(tmp_path, bad):
    with pytest.raises(ValueError, match='INVALID_TRAIN_PATH'):
        _image_and_label_dirs({'train': bad}, 'train', tmp_path)
