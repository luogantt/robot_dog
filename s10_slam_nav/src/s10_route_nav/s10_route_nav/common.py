"""Pure route/navigation helpers kept independent from ROS for unit testing."""

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import yaml


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def approach(current: float, target: float, max_delta: float) -> float:
    if target > current:
        return min(target, current + max_delta)
    return max(target, current - max_delta)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Checkpoint:
    checkpoint_id: str
    x: float
    y: float
    yaw: float = 0.0
    position_tolerance: float = 0.35
    yaw_tolerance_deg: float = 180.0
    required: bool = True


@dataclass
class Route:
    frame_id: str
    name: str
    loop: bool
    max_corridor_error: float
    warn_corridor_error: float
    controller: Dict[str, float]
    path: List[PathPoint]
    checkpoints: List[Checkpoint] = field(default_factory=list)
    cumulative_distance: List[float] = field(default_factory=list)


def _finite_float(value, label: str) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f'{label} must be finite')
    return converted


def cumulative_distances(path: Sequence[PathPoint]) -> List[float]:
    cumulative = [0.0]
    for previous, current in zip(path, path[1:]):
        cumulative.append(cumulative[-1] + math.hypot(current.x - previous.x,
                                                      current.y - previous.y))
    return cumulative


def load_route(filename: str) -> Route:
    route_path = Path(filename).expanduser().resolve()
    if not route_path.is_file():
        raise FileNotFoundError(f'route file does not exist: {route_path}')
    with route_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}

    raw_path = document.get('path', [])
    points: List[PathPoint] = []
    for index, item in enumerate(raw_path):
        if isinstance(item, dict):
            x, y = item.get('x'), item.get('y')
            yaw = item.get('yaw', 0.0)
        else:
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                raise ValueError(f'path[{index}] must contain at least x and y')
            x, y = item[0], item[1]
            yaw = item[2] if len(item) > 2 else 0.0
        points.append(PathPoint(_finite_float(x, f'path[{index}].x'),
                                _finite_float(y, f'path[{index}].y'),
                                _finite_float(yaw, f'path[{index}].yaw')))
    if len(points) < 2:
        raise ValueError('route path must contain at least two points')

    checkpoints: List[Checkpoint] = []
    for index, item in enumerate(document.get('checkpoints', [])):
        if not isinstance(item, dict):
            raise ValueError(f'checkpoints[{index}] must be a mapping')
        checkpoints.append(Checkpoint(
            checkpoint_id=str(item.get('id', f'P{index + 1:02d}')),
            x=_finite_float(item.get('x'), f'checkpoints[{index}].x'),
            y=_finite_float(item.get('y'), f'checkpoints[{index}].y'),
            yaw=_finite_float(item.get('yaw', 0.0), f'checkpoints[{index}].yaw'),
            position_tolerance=_finite_float(
                item.get('position_tolerance', 0.35),
                f'checkpoints[{index}].position_tolerance'),
            yaw_tolerance_deg=_finite_float(
                item.get('yaw_tolerance_deg', 180.0),
                f'checkpoints[{index}].yaw_tolerance_deg'),
            required=bool(item.get('required', True)),
        ))

    controller = {str(key): _finite_float(value, f'controller.{key}')
                  for key, value in (document.get('controller', {}) or {}).items()}
    route = Route(
        frame_id=str(document.get('frame_id', 'map')),
        name=str(document.get('route_name', route_path.stem)),
        loop=bool(document.get('loop', False)),
        max_corridor_error=_finite_float(document.get('max_corridor_error', 0.60),
                                         'max_corridor_error'),
        warn_corridor_error=_finite_float(document.get('warn_corridor_error', 0.40),
                                          'warn_corridor_error'),
        controller=controller,
        path=points,
        checkpoints=checkpoints,
    )
    if route.warn_corridor_error >= route.max_corridor_error:
        raise ValueError('warn_corridor_error must be smaller than max_corridor_error')
    route.cumulative_distance = cumulative_distances(points)
    return route


def nearest_path_index(path: Sequence[PathPoint], x: float, y: float,
                       start: int = 0, end: int = None) -> Tuple[int, float]:
    if not path:
        raise ValueError('path is empty')
    lower = max(0, start)
    upper = len(path) if end is None else min(len(path), end)
    if lower >= upper:
        lower, upper = 0, len(path)
    best_index = lower
    best_distance = math.inf
    for index in range(lower, upper):
        distance = math.hypot(path[index].x - x, path[index].y - y)
        if distance < best_distance:
            best_index, best_distance = index, distance
    return best_index, best_distance


def lookahead_path_index(cumulative: Sequence[float], nearest_index: int,
                         lookahead: float) -> int:
    target_distance = cumulative[nearest_index] + max(0.0, lookahead)
    for index in range(nearest_index, len(cumulative)):
        if cumulative[index] >= target_distance:
            return index
    return len(cumulative) - 1


def checkpoint_stations(route: Route) -> List[float]:
    stations = []
    search_start = 0
    for checkpoint in route.checkpoints:
        index, _ = nearest_path_index(route.path, checkpoint.x, checkpoint.y, search_start)
        stations.append(route.cumulative_distance[index])
        search_start = index
    return stations


def pure_pursuit_target(robot_x: float, robot_y: float, robot_yaw: float,
                        target: PathPoint) -> Tuple[float, float, float, float]:
    dx = target.x - robot_x
    dy = target.y - robot_y
    local_x = math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
    local_y = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy
    distance_sq = max(dx * dx + dy * dy, 1e-6)
    curvature = 2.0 * local_y / distance_sq
    heading_error = wrap_angle(math.atan2(dy, dx) - robot_yaw)
    return local_x, local_y, curvature, heading_error
