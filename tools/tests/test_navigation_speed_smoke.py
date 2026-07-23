import math
import yaml

from tools.navigation_speed_smoke import (
    collision_count, make_open_environment, path_segments, segment_intersects_box,
    segments_length)


def test_segment_collision_includes_crossing_and_excludes_clear_segment():
    box = ((1.0, 1.0, 1.0), (2.0, 2.0, 2.0))
    assert segment_intersects_box((0.0, 1.5, 1.5), (3.0, 1.5, 1.5), box)
    assert not segment_intersects_box((0.0, 0.0, 0.0), (0.5, 0.5, 0.5), box)
    assert collision_count(
        [(0.0, 1.5, 1.5), (3.0, 1.5, 1.5), (4.0, 1.5, 1.5)], [box]) == 1


def test_segments_length_accepts_path_history_shape():
    segments = [
        {"points": [[0.0, 0.0, 0.0], [3.0, 4.0, 0.0]]},
        {"points": [[3.0, 4.0, 0.0], [3.0, 4.0, 2.0]]},
    ]
    assert math.isclose(segments_length(segments), 7.0)


def test_open_environment_omits_untyped_empty_obstacle_list(tmp_path):
    path = make_open_environment(tmp_path)
    document = yaml.safe_load(path.read_text())
    parameters = next(iter(document.values()))["ros__parameters"]
    assert "obstacles" not in parameters


def test_path_segments_reads_recorder_schema(tmp_path):
    path = tmp_path / "paths.json"
    path.write_text('{"planned_segments":[{"mission_time_s":3.5,"points":[[0,0,1],[1,0,1]]}]}')
    assert path_segments(path, "planned")[0]["mission_time_s"] == 3.5
