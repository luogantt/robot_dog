import math
from pathlib import Path

import yaml

from s10_route_nav.common import (
    PathPoint,
    cumulative_distances,
    load_route,
    lookahead_path_index,
    nearest_path_index,
    pure_pursuit_target,
    wrap_angle,
)


def test_wrap_angle():
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi, abs_tol=1e-9)


def test_path_queries():
    path = [PathPoint(0.0, 0.0), PathPoint(1.0, 0.0), PathPoint(2.0, 0.0)]
    cumulative = cumulative_distances(path)
    assert cumulative == [0.0, 1.0, 2.0]
    assert nearest_path_index(path, 1.1, 0.2)[0] == 1
    assert lookahead_path_index(cumulative, 0, 1.1) == 2


def test_pure_pursuit_left_target():
    target = PathPoint(1.0, 1.0)
    _, local_y, curvature, heading = pure_pursuit_target(0.0, 0.0, 0.0, target)
    assert local_y > 0.0
    assert curvature > 0.0
    assert heading > 0.0


def test_load_route(tmp_path: Path):
    filename = tmp_path / 'route.yaml'
    filename.write_text(yaml.safe_dump({
        'frame_id': 'map',
        'warn_corridor_error': 0.4,
        'max_corridor_error': 0.6,
        'path': [[0, 0, 0], [1, 0, 0]],
        'checkpoints': [{'id': 'P01', 'x': 1, 'y': 0}],
    }), encoding='utf-8')
    route = load_route(str(filename))
    assert route.frame_id == 'map'
    assert route.cumulative_distance[-1] == 1.0
    assert route.checkpoints[0].checkpoint_id == 'P01'
