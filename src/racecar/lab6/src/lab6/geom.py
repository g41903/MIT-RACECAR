"""vectorized geometry utilities"""
import numpy as np
import warnings

def slerp(p0, p1, t, omega=None):
    """ Spherical interpolation. omega can be pre-calculated for efficiency """
    if omega is None:
        omega = np.arccos(
            p0.dot(p1) / np.linalg.norm(p0) * np.linalg.norm(p1)
        )
    return 1/np.sin(omega) * (
        np.sin((1-t)*omega)*p0 + np.sin(t*omega)*p1
    )

def vdot(a, b):
    """ Broadcasting dot product for vectors"""
    return np.einsum('...i,...i->...', a, b)

def v2cross(a, b):
    """ Builtin cross is not broadcasting in this version of numpy """
    return a[...,0]*b[...,1] - a[...,1]*b[...,0]

def signed_angle(v1, v2):
    return np.arctan2(
        v2cross(v1, v2),
        vdot(v1, v2)
    )

arc_dtype = np.dtype([
    ('center', np.float64, 2),
    ('angle', np.float64),
    ('radius', np.float64),
    ('center_to_src', np.float64, 2),
    ('center_to_dest', np.float64, 2),
])

def connect_pose_to_point(src, dest):
    """
    Connect a pose to a point with a circular arc.
    Vectorized in `src`, so can make multiple simulataneous connections

    src:  an np.recarray of pose_dtypes
    dest: an np.array of shape (,2)

    Returns a np.recarray matching the shape of src, with the following fields:
        center
        angle           Measured counterclockwise
        radius          Positive means the circle center is to the left
        center_to_src   src - center
        center_to_dest  dest - center
    """
    # the data structure we return
    arc = np.recarray(src.shape, dtype=arc_dtype)

    # direction vector pointing to the center of rotation
    left_dirs = np.array([-np.sin(src.theta), np.cos(src.theta)])
    left_dirs = np.rollaxis(left_dirs, 0, left_dirs.ndim)

    diffs = dest - src.xy
    diffs_d_dirs = vdot(diffs, left_dirs)

    arc.radius = np.where(
        diffs_d_dirs==0, 0,
        0.5 * vdot(diffs, diffs) / diffs_d_dirs
    )

    arc.center_to_src = -left_dirs*arc.radius[...,np.newaxis]
    arc.center = src.xy - arc.center_to_src
    arc.center_to_dest = dest - arc.center

    # 2D angle in [-pi pi]
    arc.angle = signed_angle(arc.center_to_src, arc.center_to_dest)

    return arc

curve_dtype = np.dtype([
    ('c1', np.float64, 2),
    ('c2', np.float64, 2),
    ('midpoint', np.float64, 2),
    ('angle1', np.float64),
    ('angle2', np.float64),
    ('r', np.float64)
])

def connect_pose_to_pose(src, dest):
    """
    Connect a pose to a pose with a pair of circular arcs.
    Fully broadcasting

    src:  an np.recarray of pose_dtypes
    dest: an single pose_dtype

    Returns a np.recarray matching the shape of src, with the following fields:
        c1              The center of the arc starting at src
        angle1          The counterclockwise angle of the arc starting at src
        c2              The center of the arc ending at dest
        angle1          The counterclockwise angle of the arc ending at dest
        radius          Positive means the center of the first arc is to the left
                        The second arcs center will be on the opposite side
        midpoint        The location where the two arcs meet
    """

    # the data structure we return
    curve = np.recarray(np.broadcast(src, dest).shape, dtype=curve_dtype)

    # direction vectors pointing towards or away from the centers of rotation
    left_src_dirs = np.array([-np.sin(src.theta), np.cos(src.theta)])
    left_src_dirs = np.rollaxis(left_src_dirs, 0, left_src_dirs.ndim)

    left_dest_dirs = np.array([-np.sin(dest.theta), np.cos(dest.theta)])
    left_dest_dirs = np.rollaxis(left_dest_dirs, 0, left_dest_dirs.ndim)

    '''
    Calculate r

    $\frac{
         (\mathbf{v}_s + \mathbf{v}_d) \cdot (\mathbf{p}_d - \mathbf{p}_s)
    }{
        \|\mathbf{v}_s + \mathbf{v}_d\|^2 - 4
    } \pm \sqrt{
        \left(\frac{
             (\mathbf{v}_s + \mathbf{v}_d) \cdot (\mathbf{p}_d - \mathbf{p}_s)
        }{
            \|\mathbf{v}_s + \mathbf{v}_d\|^2 - 4
        }\right)^2
        -  \frac{\|\mathbf{p}_d - \mathbf{p}_s\|^2}{\|\mathbf{v}_s + \mathbf{v}_d\|^2 - 4}
    }$
    '''

    diffs = dest.xy - src.xy
    dirs_sum = left_dest_dirs + left_src_dirs
    signs = np.array([1, -1]).reshape((-1,) + (1,) * src.ndim)

    # compute these once
    diffs_d_diffs = vdot(diffs, diffs)
    diffs_d_dirs = vdot(diffs, dirs_sum)

    # this is a term that apperas
    some_norm = vdot(dirs_sum, dirs_sum) - 4

    aterm = diffs_d_dirs / some_norm

    # solve the quadratic, and choose the smaller magnitude of the two solutions
    rs = aterm + signs * np.sqrt(aterm*aterm - diffs_d_diffs / some_norm)
    rs = np.where(abs(rs[0]) < abs(rs[1]), rs[0], rs[1])

    # unless there's no quadratic term
    rs_noquad = 0.5 * diffs_d_diffs / diffs_d_dirs
    rs_noquad = np.where(np.isclose(diffs_d_dirs, 0), 0, rs_noquad)  # prevent /0
    rs = np.where(
        np.isclose(some_norm, 0),
        rs_noquad,
        rs
    )

    # choose the smaller magnitude of the two solutions
    curve.r = rs

    curve.c1 = src.xy + left_src_dirs * curve.r[...,np.newaxis]
    curve.c2 = dest.xy - left_dest_dirs * curve.r[...,np.newaxis]
    curve.midpoint = 0.5*(curve.c1 + curve.c2)

    curve.angle1 = signed_angle(src.xy - curve.c1, curve.midpoint - curve.c1)
    curve.angle2 = signed_angle(curve.midpoint - curve.c2, dest.xy - curve.c2)

    bad = ~np.isfinite(curve.angle1) | ~np.isfinite(curve.angle2) | ~np.isfinite(curve.r)
    if bad.any():
        src, dest = np.broadcast_arrays(src, dest, subok=True)
        warnings.warn(RuntimeWarning,
            "Non-finite result encountered finding a curve between {} and {}: ".format(
                src[bad], dest[bad], curve[bad]
            )
        )

    return curve


# ok, this is not really geometry...
def argfirst(boolarr):
    """
    Return the index of the first true value, or the end of the array if no
    such value is found
    """
    if not boolarr.any():
        return len(boolarr)
    else:
        return np.argmax(boolarr)
