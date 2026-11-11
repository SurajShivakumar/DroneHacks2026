import subprocess

from PIL import Image
import pytest

from dronebench.recording import ImageSequence, encode_preview


def make_sequence(tmp_path):
    image = tmp_path / "input.png"
    Image.new("RGB", (16, 16), "red").save(image)
    sequence = ImageSequence(tmp_path / "frames", topic="/camera/image",
                             clock_domain="simulation", provenance={"test": True})
    return sequence, image


def test_source_timestamps_and_exclusive_output(tmp_path):
    sequence, image = make_sequence(tmp_path)
    sequence.append(image, 100)
    sequence.append(image, 1000000100)
    result = sequence.close()
    assert result["source_span_seconds"] == 1.
    assert result["finish_time_seconds"] is None
    assert result["completion_evaluated"] is False
    assert result["frames"][0]["sha256"] == result["frames"][1]["sha256"]
    with pytest.raises(FileExistsError):
        ImageSequence(sequence.output, topic="x", clock_domain="simulation", provenance={})
    with pytest.raises(ValueError):
        sequence.append(image, 2000000000)


def test_bad_stamps_do_not_add_frames(tmp_path):
    sequence, image = make_sequence(tmp_path)
    sequence.append(image, 100)
    for stamp in [100, 99, -1, 101., True]:
        with pytest.raises(ValueError):
            sequence.append(image, stamp)
    assert len(list(sequence.output.glob("*.png"))) == 1


def test_hash_mismatch_rejected_before_encoding(tmp_path):
    sequence, image = make_sequence(tmp_path)
    sequence.append(image, 0)
    sequence.close()
    (sequence.output / "frame_000000.png").write_bytes(b"changed")
    with pytest.raises(ValueError, match="hash mismatch"):
        encode_preview(sequence.output, tmp_path / "video")
    assert not (tmp_path / "video").exists()


def test_dimension_change_rejected(tmp_path):
    sequence, image = make_sequence(tmp_path)
    sequence.append(image, 0)
    Image.new("RGB", (20, 16)).save(image)
    with pytest.raises(ValueError, match="dimensions changed"):
        sequence.append(image, 1)


def test_real_encoder_and_decoder(tmp_path):
    imageio_ffmpeg = pytest.importorskip("imageio_ffmpeg")
    executable = imageio_ffmpeg.get_ffmpeg_exe()
    sequence, image = make_sequence(tmp_path)
    for stamp in (0, 100000000, 900000000):
        sequence.append(image, stamp)
    sequence.close()
    output = tmp_path / "video"
    result = encode_preview(sequence.output, output, fps=10, ffmpeg=executable)
    assert result["source_timestamps_ns"] == [0, 100000000, 900000000]
    assert result["finish_time_seconds"] is None
    decoded = subprocess.run([executable, "-v", "error", "-i", str(output / "preview.mp4"),
                              "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                             capture_output=True, check=True)
    assert len(decoded.stdout) == 3 * 16 * 16 * 3
    with pytest.raises(FileExistsError):
        encode_preview(sequence.output, output, ffmpeg=executable)
