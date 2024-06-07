import numpy as np


def distance(start, end) -> float:
	return np.linalg.norm(np.array(start) - np.array(end))


def lerp(start: list, end: list, n: int = 500) -> tuple:
	start = np.array(start)
	end = np.array(end)
	# Create a linear interpolation factor array
	t = np.linspace(0.0, 1.0, n)
	x = ((1 - t) * start[0]) + t * end[0]
	y = ((1 - t) * start[1]) + t * end[1]
	z = ((1 - t) * start[2]) + t * end[2]
	return x.tolist(), y.tolist(), z.tolist()
