"""A small 3D frame solver for cages and rod structures (Euler-Bernoulli beams, 6 dof a node, linear static).

Members carry E, A, I (both axes), J; joints are rigid (an upper bound on how a TPU node holds rotation; the buckling check
uses pinned ends, k = 1, the conservative side). Units mm, N, MPa.

    from frame_model import Frame
    f = Frame(); a = f.node(p); f.member(a, b, sec); f.fix(c)
    u = f.solve({node: force_xyz}); f.member_forces(u) -> list of dict(N, M, V, ...)
"""
import numpy as np
from scipy.sparse import lil_matrix, csr_matrix
from scipy.sparse.linalg import spsolve


def tube(od, id_, E=100e3, G=None):
    """a round section: E MPa (carbon pultruded 100 GPa, E), area, I, J"""
    A = np.pi / 4 * (od ** 2 - id_ ** 2); I = np.pi / 64 * (od ** 4 - id_ ** 4)
    return dict(E=E, G=G or E / 2.6, A=A, I=I, J=2 * I, c=od / 2, od=od, id=id_)


class Frame:
    def __init__(self):
        self.X, self.M, self.fixed = [], [], set()

    def node(self, p, tol=0.5):
        p = np.asarray(p, float)
        for i, q in enumerate(self.X):
            if np.linalg.norm(q - p) < tol: return i
        self.X.append(p); return len(self.X) - 1

    def member(self, a, b, sec, name=""):
        self.M.append(dict(a=a, b=b, sec=sec, name=name))

    def fix(self, i):
        self.fixed.add(i)

    def _k(self, m):
        Xa, Xb = self.X[m["a"]], self.X[m["b"]]; d = Xb - Xa; L = np.linalg.norm(d); s = m["sec"]
        E, G, A, I, J = s["E"], s["G"], s["A"], s["I"], s["J"]
        k = np.zeros((12, 12))
        ea = E * A / L; gj = G * J / L; a1 = 12 * E * I / L ** 3; a2 = 6 * E * I / L ** 2; a3 = 4 * E * I / L; a4 = 2 * E * I / L
        k[0, 0] = k[6, 6] = ea; k[0, 6] = k[6, 0] = -ea
        k[3, 3] = k[9, 9] = gj; k[3, 9] = k[9, 3] = -gj
        for (v, t, sg) in ((1, 5, 1), (2, 4, -1)):                     # bending in local y (rot z) and local z (rot y)
            k[v, v] = k[v + 6, v + 6] = a1; k[v, v + 6] = k[v + 6, v] = -a1
            k[v, t] = k[t, v] = sg * a2; k[v, t + 6] = k[t + 6, v] = sg * a2
            k[v + 6, t] = k[t, v + 6] = -sg * a2; k[v + 6, t + 6] = k[t + 6, v + 6] = -sg * a2
            k[t, t] = k[t + 6, t + 6] = a3; k[t, t + 6] = k[t + 6, t] = a4
        ex = d / L; ref = np.array([0, 0, 1.0]) if abs(ex[2]) < 0.9 else np.array([1.0, 0, 0])
        ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
        R = np.vstack([ex, ey, ez]); T = np.zeros((12, 12))
        for q in range(4): T[3 * q:3 * q + 3, 3 * q:3 * q + 3] = R
        return T.T @ k @ T, T, k, L

    def solve(self, loads):
        n = len(self.X); K = lil_matrix((6 * n, 6 * n)); self._cache = []
        for m in self.M:
            kg, T, kl, L = self._k(m); self._cache.append((T, kl, L))
            idx = list(range(6 * m["a"], 6 * m["a"] + 6)) + list(range(6 * m["b"], 6 * m["b"] + 6))
            for i_, gi in enumerate(idx):
                for j_, gj in enumerate(idx):
                    if kg[i_, j_]: K[gi, gj] += kg[i_, j_]
        F = np.zeros(6 * n)
        for i, f in loads.items(): F[6 * i:6 * i + 3] += f
        free = np.array([d for d in range(6 * n) if d // 6 not in self.fixed])
        K = csr_matrix(K); Kff = K[free][:, free]
        Kff = Kff + csr_matrix((np.full(len(free), 1e-9 * Kff.diagonal().mean()), (range(len(free)), range(len(free)))))
        u = np.zeros(6 * n); u[free] = spsolve(Kff, F[free])
        return u

    def member_forces(self, u):
        out = []
        for m, (T, kl, L) in zip(self.M, self._cache):
            ue = np.concatenate([u[6 * m["a"]:6 * m["a"] + 6], u[6 * m["b"]:6 * m["b"] + 6]])
            f = kl @ (T @ ue); s = m["sec"]
            N = -f[0]                                                      # + tension
            Mb = max(np.hypot(f[4], f[5]), np.hypot(f[10], f[11]))
            sig = abs(N) / s["A"] + Mb * s["c"] / s["I"]
            Pcr = np.pi ** 2 * s["E"] * s["I"] / L ** 2                    # pinned ends (k = 1)
            out.append(dict(name=m["name"], a=m["a"], b=m["b"], L=L, N=N, M=Mb, sigma=sig, Pcr=Pcr,
                            buckle=(-N / Pcr) if N < 0 else 0.0, f_end_a=f[:3], f_end_b=f[6:9]))
        return out
