import numpy as np
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# points lying on corners of a square
square = np.array([
	[0, 0, 0],
	[0, 1, 0],
	[1, 1, 0],
	[1, 0, 0]
])

def plot_cube(ax, loc, color, size):

	permute = np.eye(3)
	square_c = square - np.array([0.5, 0.5, 0])
	square_c *= size
	offset = np.array([0, 0, 0.5]) * size

	faces = []
	for i in range(3):
		face = loc + square_c.dot(permute.T)
		off = offset.dot(permute.T)
		faces += [face + off, face-off]
		permute = np.roll(permute, 1, axis=0)

	poly = Poly3DCollection(faces)
	poly.set_facecolor(color)
	ax.add_collection3d(poly)
	return poly


def plot_voxels(ax, filled, color=None, step=1):
	"""
	Plot a set of filled voxels where `filled` is True
	"""
	# check dimensions, and deal with a single color
	assert filled.ndim == 3
	if color is None:
		color = next(ax._get_lines.color_cycle)

	if np.ndim(color) <= 1:
		color, _ = np.broadcast_arrays(
			color,
			filled[np.index_exp[...] + np.index_exp[np.newaxis] * np.ndim(color)]
		)
	else:
		assert np.ndim(color) >= 3 and np.shape(color)[:3] == filled.shape

	ax.auto_scale_xyz(
		[0, step*filled.shape[0]],
		[0, step*filled.shape[1]],
		[0, step*filled.shape[2]]
	)

	def edge_found(surface, val):
		points = surface * step
		poly = Poly3DCollection([points])
		poly.set_facecolor(val)
		ax.add_collection3d(poly)

	# cyclic permutation matrix
	permute = np.eye(3)

	for i in range(3):
		# find the set of ranges to iterate over
		pc, qc, rc = permute.T.dot(filled.shape[:3])
		pinds = np.arange(pc)
		qinds = np.arange(qc)
		rinds = np.arange(rc)

		for p in pinds:
			for q in qinds:
				# draw lower faces
				p0 = permute.dot([p, q, 0])
				i0 = tuple(p0)
				if filled[i0]:
					edge_found(p0 + square.dot(permute.T), color[i0])

				# draw middle faces
				for r1, r2 in zip(rinds, rinds[1:]):
					p1 = permute.dot([p, q, r1])
					p2 = permute.dot([p, q, r2])

					i1 = tuple(p1)
					i2 = tuple(p2)

					if filled[i1] and not filled[i2]:
						edge_found(p2 + square.dot(permute.T), color[i1])
					elif not filled[i1] and filled[i2]:
						edge_found(p2 + square.dot(permute.T), color[i2])

				# draw upper faces
				pk = permute.dot([p, q, rc-1])
				pk2 = permute.dot([p, q, rc])
				ik = tuple(pk)
				if filled[ik]:
					edge_found(pk2 + square.dot(permute.T), color[ik])

		# permute the matrix
		permute = np.roll(permute, 1, axis=0)

