import numpy as np

from geometry.util import find_nearest, shape_bounds
from geometry.shapes import Point

from .types import VectorMap


class RasterMap:
    def __init__(self, offset_x: float, offset_y: float, scale: float, data: np.ndarray, default_value: float = 0.0) -> None:
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.scale = scale
        self.data = data
        self.default_value = default_value

    def get(self, x: float, y: float) -> float:
        x_index = int((x - self.offset_x) / self.scale)
        y_index = int((y - self.offset_y) / self.scale)

        if 0 <= x_index < self.data.shape[1] and 0 <= y_index < self.data.shape[0]:
            return self.data[y_index, x_index]
        else:
            return self.default_value

    def get_many(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        x_indices = ((xs - self.offset_x) / self.scale).astype(int)
        y_indices = ((ys - self.offset_y) / self.scale).astype(int)

        valid_mask = (
            (x_indices >= 0) & (x_indices < self.data.shape[1]) &
            (y_indices >= 0) & (y_indices < self.data.shape[0])
        )

        result = np.full(xs.shape, self.default_value, dtype="f")
        result[valid_mask] = self.data[y_indices[valid_mask], x_indices[valid_mask]]
        return result


def make_distance_map(map: VectorMap, scale: float, padding: float) -> RasterMap:
    x1, y1, x2, y2 = shape_bounds(map)
    print("Bounds:", x1, y1, x2, y2)
    offset_x = x1 - padding
    offset_y = y1 - padding
    width = int((x2 - x1 + 2 * padding) / scale)
    height = int((y2 - y1 + 2 * padding) / scale)
    print("Width:", width)
    print("Height:", height)
    print("Offset:", offset_x, offset_y)

    data = np.full((height, width), np.inf, dtype="f")
    for x in range(width):
        for y in range(height):
            point = Point(
                x=offset_x + x * scale,
                y=offset_y + y * scale,
            )
            _, dist = find_nearest(map, point)
            data[y, x] = dist

    return RasterMap(
        offset_x=offset_x,
        offset_y=offset_y,
        scale=scale,
        data=data,
        default_value=float("inf"),
    )


def save_raster_map(distance_map: RasterMap, path: str) -> None:
    np.savez_compressed(
        path,
        offset_x=distance_map.offset_x,
        offset_y=distance_map.offset_y,
        scale=distance_map.scale,
        data=distance_map.data,
        default_value=distance_map.default_value,
    )


def load_raster_map(path: str) -> RasterMap:
    npz = np.load(path)
    return RasterMap(
        offset_x=float(npz["offset_x"]),
        offset_y=float(npz["offset_y"]),
        scale=float(npz["scale"]),
        data=npz["data"],
        default_value=float(npz["default_value"]),
    )
