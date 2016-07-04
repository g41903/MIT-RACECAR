#! python2
"""
Usage:
	plot.py FILE.npy [--threshold | --threshold=<t>] [--bits=<b>]
"""

import numpy as np
import scipy as sc
from matplotlib import pyplot as plt
import matplotlib

import scipy.spatial
import scipy.io

import sys
import os
import re

from visual_servoing.plot import *


threshold = False
threshold_val = 128
fname = None
BITS = 3
for arg in sys.argv[1:]:
	m = re.match(r'--threshold=(\d+)', arg)
	m2 = re.match(r'--bits=(\d+)', arg)
	if m:
		threshold_val = int(m.group(1))
		threshold = True
	elif m2:
		BITS = int(m2.group(1))
		assert 0 < BITS <= 8
	elif '--threshold' == arg:
		threshold = True
	elif fname is None:
		fname = arg
	else:
		raise SystemExit("Unrecognized argument {}\n{}".format(arg, __doc__))

basename = os.path.splitext(fname)[0]

data = np.load(fname)[...,::-1]

# discretize the image
data_disc = data >> (8 - BITS)

data_c = np.zeros((1<<BITS, 1<<BITS, 1<<BITS))

idxs = tuple(data_disc.T)
np.add.at(data_c, idxs, 1)
data_c /= np.max(data_c)

data_disc = np.vstack({tuple(row) for row in data_disc})

# reconstruct colors from those points
data_colors = (data_disc + 0.5)*2**(8 - BITS) / 255.0

MAX = 256
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')
ax.axis('scaled')
ax.set(
	xlabel='R', ylabel='G', zlabel='B',
	xlim=[0, MAX], ylim=[0, MAX], zlim=[0, MAX],
	xticks=np.linspace(0, MAX, 5),
	yticks=np.linspace(0, MAX, 5),
	zticks=np.linspace(0, MAX, 5),
)
ax.patch.set_visible(False)
matplotlib.rc('font', size=18)

ax.tick_params(axis='x', colors='red')
ax.xaxis.label.set_color('red')
ax.xaxis.line.set_color('red')
ax.tick_params(axis='y', colors='green')
ax.yaxis.label.set_color('green')
ax.yaxis.line.set_color('green')
ax.tick_params(axis='z', colors='blue')
ax.zaxis.label.set_color('blue')
ax.zaxis.line.set_color('blue')

plt.setp([
	ax.w_xaxis,
	ax.w_yaxis,
	ax.w_zaxis,
], pane_color=[1, 1, 1, 0])

step = (1 << (8 - BITS))

if False:
	# plot convex hull
	hull = scipy.spatial.ConvexHull(data)
	ax.plot_trisurf(data[:, 0], data[:, 1], data[:, 2],triangles=hull.simplices, color='orange')

elif False:
	# plot a scatter of the points
	ax.scatter(data_color[:, 0], data_color[:, 1], data_color[:, 2], marker='x', c=data_color)

else:
	# create an occupancy grid to plot voxels
	data_occ = np.empty((1<<BITS, 1<<BITS, 1<<BITS), dtype=object)
	for idx, color in zip(data_disc, data_colors):
		data_occ[tuple(idx)] = color

	if threshold:
		data_occ2 = data_occ.copy()
		matching = (data_c ** 0.5 > (threshold_val/255.0))
		cut = ~matching & (data_occ != np.array(None))
		cut_loc = step * (np.indices(data_c.shape)[:,cut] + 0.5)

		cut_col = np.vstack(data_occ[cut])

		for loc, col in zip(cut_loc.T, cut_col):
			plot_cube(ax, loc, col, step/2).set_edgecolor('w')
		data_occ[cut] = None

		lookup_out = '{}-lookup-{}-{}.npy'.format(basename, BITS, threshold_val)
		np.save(lookup_out, matching)
		print "Saved lookup table to {}".format(lookup_out)

	plot_voxels(ax, data_occ != np.array(None), color=data_occ, step=step)

fig.tight_layout()
fout = '{}-plot-{}-{}.png'.format(basename, BITS, threshold_val)
fig.savefig(fout, transparent=True)
print "Saved plot to {}".format(fout)


plt.show()
