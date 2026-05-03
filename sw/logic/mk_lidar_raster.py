from map.raster import make_distance_map, save_raster_map
from map.loader import load_world_from_json

from PIL import Image

import numpy as np

vec = load_world_from_json("data/map_bear_rescue.json")
distance_map = make_distance_map(vec, scale=0.003, padding=0.5)

sigma = 0.1
distance_map.data = (1.0 / (sigma * (2.0 * np.pi) ** 0.5)) * np.exp(-0.5 * ((distance_map.data) / sigma) ** 2) + 0.2
distance_map.default_value = 0.2


save_raster_map(distance_map, "data/map_lidar_likelihood.npz")


img = Image.fromarray((distance_map.data * 32).clip(0, 255).astype('uint8'))
img.save("data/map_lidar_likelihood.png")

print(distance_map.data)


gradient = np.gradient(distance_map.data)
gradient_magnitude = np.sqrt(gradient[0] ** 2 + gradient[1] ** 2)
gradient_magnitude = gradient_magnitude / np.abs(distance_map.data)
gradient_magnitude = (gradient_magnitude / np.max(gradient_magnitude) * 255).clip(0, 255).astype('uint8')
gradient_img = Image.fromarray(gradient_magnitude)
gradient_img.save("data/map_lidar_likelihood_gradient.png")
