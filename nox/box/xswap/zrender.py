"""Tiny orthographic z-buffer renderer (numpy) with flat shading and depth / part outlines, for honest assembly images."""
import math
import numpy as np

def view_matrix(elev, azim):
    a, e = math.radians(azim), math.radians(elev)
    # camera looks along -d; d = direction from target to camera
    d = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    up = np.array([0, 0, 1.0]) if abs(d[2]) < 0.99 else np.array([0, 1.0, 0])
    r = np.cross(up, d); r /= np.linalg.norm(r); u = np.cross(d, r)
    return np.stack([r, u, d])          # rows: screen x, screen y, depth (toward camera)

def render(parts, elev=20, azim=-60, size=900, pad=0.06, bounds=None, light=(0.35, -0.5, 0.8), bg=(1, 1, 1)):
    """parts: list of (trimesh, rgb). Returns an (size, size, 3) float image."""
    V = view_matrix(elev, azim)
    L = np.array(light, float); L /= np.linalg.norm(L)
    allp = np.concatenate([t.vertices for t, _ in parts]) if bounds is None else np.array(bounds)
    P = allp @ V.T
    lo, hi = P.min(0), P.max(0)
    span = max(hi[0] - lo[0], hi[1] - lo[1]) * (1 + 2 * pad)
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    s = size / span
    img = np.ones((size, size, 3)) * np.array(bg)
    zb = np.full((size, size), -np.inf); idb = np.full((size, size), -1)
    for pid, (t, rgb) in enumerate(parts):
        tri = t.vertices[t.faces] @ V.T                      # (n, 3, 3) screen coords
        X = (tri[..., 0] - cx) * s + size / 2; Y = size / 2 - (tri[..., 1] - cy) * s; Z = tri[..., 2]
        n = t.face_normals
        shade = 0.30 + 0.70 * np.clip(n @ L, 0, 1) * 0.85 + 0.15 * np.clip(n @ V[2], 0, 1)
        col = np.clip(np.array(rgb)[None, :] * shade[:, None], 0, 1)
        x0 = np.clip(np.floor(X.min(1)).astype(int), 0, size - 1); x1 = np.clip(np.ceil(X.max(1)).astype(int), 0, size - 1)
        y0 = np.clip(np.floor(Y.min(1)).astype(int), 0, size - 1); y1 = np.clip(np.ceil(Y.max(1)).astype(int), 0, size - 1)
        for i in range(len(tri)):
            if x1[i] < x0[i] or y1[i] < y0[i]: continue
            xs = np.arange(x0[i], x1[i] + 1) + 0.5; ys = np.arange(y0[i], y1[i] + 1) + 0.5
            gx, gy = np.meshgrid(xs, ys)
            (ax_, bx, cx_), (ay, by, cy_) = X[i], Y[i]
            den = (by - cy_) * (ax_ - cx_) + (cx_ - bx) * (ay - cy_)
            if abs(den) < 1e-9: continue
            w0 = ((by - cy_) * (gx - cx_) + (cx_ - bx) * (gy - cy_)) / den
            w1 = ((cy_ - ay) * (gx - cx_) + (ax_ - cx_) * (gy - cy_)) / den
            w2 = 1 - w0 - w1
            inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
            if not inside.any(): continue
            z = w0 * Z[i, 0] + w1 * Z[i, 1] + w2 * Z[i, 2]
            sl = (slice(y0[i], y1[i] + 1), slice(x0[i], x1[i] + 1))
            cur = zb[sl]
            upd = inside & (z > cur)
            if upd.any():
                cur[upd] = z[upd]; zb[sl] = cur
                im = img[sl]; im[upd] = col[i]; img[sl] = im
                idm = idb[sl]; idm[upd] = pid; idb[sl] = idm
    # outlines: part boundaries and depth jumps
    edge = np.zeros((size, size), bool)
    for dy, dx in ((0, 1), (1, 0)):
        a = idb; b = np.roll(idb, (dy, dx), (0, 1))
        za = zb; zb2 = np.roll(zb, (dy, dx), (0, 1))
        jump = np.abs(np.nan_to_num(za - zb2, posinf=1e3, neginf=-1e3)) > 1.5
        edge |= (a != b) | jump
    edge &= (idb >= 0) | (np.roll(idb, 1, 0) >= 0) | (np.roll(idb, 1, 1) >= 0)
    img[edge] = img[edge] * 0.35
    return img
