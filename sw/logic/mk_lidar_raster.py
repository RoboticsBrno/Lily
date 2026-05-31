from params import ROBOT_BODY_RADIUS
from map.raster import make_distance_map, save_raster_map
from map.loader import load_world_from_json

from PIL import Image

import numpy as np

vec = load_world_from_json("data/map_bear_rescue.json")
distance_map = make_distance_map(vec, scale=0.003, padding=0.5)


# for sensor model
sigma = 0.1
baseline = 0.05

likelyhood_map = distance_map.copy()
likelyhood_map.data = (1.0 / (sigma * (2.0 * np.pi) ** 0.5)) * np.exp(-0.5 * ((np.abs(likelyhood_map.data) + 0.1) / sigma) ** 2) + baseline
likelyhood_map.default_value = baseline


save_raster_map(likelyhood_map, "data/map_lidar_likelihood.npz")


img = Image.fromarray((likelyhood_map.data * 600).clip(0, 255).astype('uint8'))
img.save("data/map_lidar_likelihood.png")

print(likelyhood_map.data)


gradient = np.gradient(likelyhood_map.data)
gradient_magnitude = np.sqrt(gradient[0] ** 2 + gradient[1] ** 2)
gradient_magnitude = gradient_magnitude / np.abs(likelyhood_map.data)
gradient_magnitude = (gradient_magnitude / np.max(gradient_magnitude) * 255).clip(0, 255).astype('uint8')
gradient_img = Image.fromarray(gradient_magnitude)
gradient_img.save("data/map_lidar_likelihood_gradient.png")


# for motion model
black_range = ROBOT_BODY_RADIUS * 0.5

motion_map = distance_map.copy()
motion_map.data = np.where(motion_map.data > black_range, 1.0, 0.0)
motion_map.default_value = 0.0

save_raster_map(motion_map, "data/map_motion_model.npz")

motion_img = Image.fromarray((motion_map.data * 255).astype('uint8'))
motion_img.save("data/map_motion_model.png")
