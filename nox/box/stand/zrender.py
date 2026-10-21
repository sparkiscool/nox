"""Small z-buffer renderer for preview images (orthographic, smooth shading split at sharp edges, silhouette ink).
render(items, elev, azim, px) -> RGBA float image. items: [(trimesh, '#rrggbb', alpha)], opaque first."""
import numpy as np, trimesh
from matplotlib.colors import to_rgb
from scipy import ndimage as ndi


def _cam(elev, azim):
    el, az = np.radians(elev), np.radians(azim)
    f = -np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])     # view direction (into scene)
    r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r); u = np.cross(r, f)
    return np.stack([r, u, -f])                                                        # rows: right, up, toward eye


def _raster(V, F, N, W, H):
    """V (n,3) screen coords (x px, y px, depth toward eye); returns depth, normal buffers."""
    zb = np.full((H, W), -np.inf); nb = np.zeros((H, W, 3))
    tri = V[F]
    x0 = np.clip(np.floor(tri[:, :, 0].min(1)).astype(int), 0, W - 1); x1 = np.clip(np.ceil(tri[:, :, 0].max(1)).astype(int), 0, W - 1)
    y0 = np.clip(np.floor(tri[:, :, 1].min(1)).astype(int), 0, H - 1); y1 = np.clip(np.ceil(tri[:, :, 1].max(1)).astype(int), 0, H - 1)
    for t in range(len(F)):
        if x1[t] < x0[t] or y1[t] < y0[t]: continue
        a, b, c = tri[t]
        den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(den) < 1e-12: continue
        xs = np.arange(x0[t], x1[t] + 1) + 0.5; ys = np.arange(y0[t], y1[t] + 1) + 0.5
        X, Y = np.meshgrid(xs, ys)
        w0 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / den
        w1 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / den
        w2 = 1 - w0 - w1
        m = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        if not m.any(): continue
        z = w0 * a[2] + w1 * b[2] + w2 * c[2]
        sl = (slice(y0[t], y1[t] + 1), slice(x0[t], x1[t] + 1))
        zs = zb[sl]; upd = m & (z > zs)
        if not upd.any(): continue
        zs[upd] = z[upd]
        n = w0[..., None] * N[F[t, 0]] + w1[..., None] * N[F[t, 1]] + w2[..., None] * N[F[t, 2]]
        nb[sl][upd] = n[upd]
    return zb, nb


def render(items, elev=20, azim=-60, px=900, pad=0.06, light=(0.45, -0.35, 0.82), bg="#f6f4ef", ink="#2a2d33",
           bounds=None, ss=2):
    R = _cam(elev, azim)
    allv = np.vstack([m.vertices for m, _, _ in items]) if bounds is None else bounds
    P = allv @ R.T; lo, hi = P.min(0), P.max(0); span = (hi - lo)[:2].max() * (1 + 2 * pad)
    W = H = px * ss; s = W / span; c = (lo + hi) / 2
    Lw = np.asarray(light, float); Lw /= np.linalg.norm(Lw); Lv = R @ Lw
    img = np.ones((H, W, 3)) * np.array(to_rgb(bg)); depth_all = np.full((H, W), -np.inf); mask_all = np.zeros((H, W), bool)
    for m, col, alpha in items:
        sm = trimesh.graph.smooth_shade(m, angle=np.radians(32)) if len(m.faces) < 400000 else m
        V = sm.vertices @ R.T; N = sm.vertex_normals @ R.T
        S = np.c_[(V[:, 0] - c[0]) * s + W / 2, H / 2 - (V[:, 1] - c[1]) * s, V[:, 2]]
        zb, nb = _raster(S, sm.faces, N, W, H)
        hit = np.isfinite(zb)
        n = nb / (np.linalg.norm(nb, axis=2, keepdims=True) + 1e-12)
        lam = np.clip((n * Lv).sum(2), 0, 1); hemi = 0.5 + 0.5 * n[..., 1]
        hv = Lv + np.array([0, 0, 1.0]); hv /= np.linalg.norm(hv)
        spec = np.clip((n * hv).sum(2), 0, 1) ** 40
        base = np.array(to_rgb(col))
        shade = base * (0.30 + 0.62 * lam[..., None] + 0.16 * hemi[..., None]) + 0.18 * spec[..., None]
        shade = np.clip(shade, 0, 1)
        if alpha >= 1:
            front = hit & (zb > depth_all)
            img[front] = shade[front]; depth_all[front] = zb[front]; mask_all |= hit
        else:
            front = hit & (zb > depth_all)
            img[front] = img[front] * (1 - alpha) + shade[front] * alpha
            behind = hit & ~front
            img[behind] = img[behind] * (1 - alpha * 0.35) + shade[behind] * alpha * 0.35
    # silhouette and crease ink from depth discontinuities of the opaque layer
    d = np.where(np.isfinite(depth_all), depth_all, depth_all[np.isfinite(depth_all)].min() - 50 if np.isfinite(depth_all).any() else 0)
    edge = (np.abs(ndi.sobel(d, 0)) + np.abs(ndi.sobel(d, 1))) > 10.0
    edge |= mask_all ^ ndi.binary_erosion(mask_all)
    edge = ndi.binary_dilation(edge, iterations=max(1, ss // 2))
    inkc = np.array(to_rgb(ink)); img[edge] = img[edge] * 0.25 + inkc * 0.75
    if ss > 1:
        img = img.reshape(px, ss, px, ss, 3).mean((1, 3))
    return img
