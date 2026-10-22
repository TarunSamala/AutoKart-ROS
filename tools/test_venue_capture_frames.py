from types import SimpleNamespace

from venue_capture_frames import FramePairer


def hdr_msg(sec, nanosec, tag):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)), tag=tag)


def test_pairer_matches_by_identical_stamp_and_stops_at_target():
    saved = []
    pairer = FramePairer(lambda i, rgb, depth: saved.append((i, rgb.tag, depth.tag)), target_pairs=2)
    pairer.on_rgb(hdr_msg(1, 0, "rgb1"))
    assert saved == []
    pairer.on_depth(hdr_msg(1, 0, "depth1"))
    assert saved == [(1, "rgb1", "depth1")]
    pairer.on_depth(hdr_msg(2, 0, "depth2"))
    pairer.on_rgb(hdr_msg(2, 0, "rgb2"))
    assert saved == [(1, "rgb1", "depth1"), (2, "rgb2", "depth2")]
    assert pairer.done
    pairer.on_rgb(hdr_msg(3, 0, "rgb3"))
    pairer.on_depth(hdr_msg(3, 0, "depth3"))
    assert len(saved) == 2


def test_pairer_ignores_mismatched_stamps():
    saved = []
    pairer = FramePairer(lambda i, rgb, depth: saved.append(i), target_pairs=1)
    pairer.on_rgb(hdr_msg(5, 0, "rgb"))
    pairer.on_depth(hdr_msg(6, 0, "depth"))
    assert saved == []
    assert not pairer.done


def test_pairer_matches_regardless_of_arrival_order():
    saved = []
    pairer = FramePairer(lambda i, rgb, depth: saved.append((rgb.tag, depth.tag)), target_pairs=1)
    pairer.on_depth(hdr_msg(7, 0, "depth-first"))
    pairer.on_rgb(hdr_msg(7, 0, "rgb-second"))
    assert saved == [("rgb-second", "depth-first")]
