"""A small 3D frame solver for cages and rod structures (Euler-Bernoulli beams, 6 dof a node, linear static).

Members carry E, A, I (both axes), J; joints are rigid (an upper bound on how a TPU node holds rotation; the buckling check
uses pinned ends, k = 1, the conservative side). Units mm, N, MPa.

    from frame_model import Frame
    f = Frame(); a = f.node(p); f.member(a, b, sec); f.fix(c)
    u = f.solve({node: force_xyz}); f.member_forces(u) -> list of dict(N, M, V, ...)
    f.modes({node: mass_g}, n) -> the lowest n natural frequencies (Hz) and mode shapes (lumped masses)
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

    def stiffness(self):
        n = len(self.X); K = lil_matrix((6 * n, 6 * n)); self._cache = []
        for m in self.M:
            kg, T, kl, L = self._k(m); self._cache.append((T, kl, L))
            idx = list(range(6 * m["a"], 6 * m["a"] + 6)) + list(range(6 * m["b"], 6 * m["b"] + 6))
            for i_, gi in enumerate(idx):
                for j_, gj in enumerate(idx):
                    if kg[i_, j_]: K[gi, gj] += kg[i_, j_]
        return csr_matrix(K)

    def solve(self, loads):
        n = len(self.X); K = self.stiffness()
        F = np.zeros(6 * n)
        for i, f in loads.items(): f = np.asarray(f, float); F[6 * i:6 * i + len(f)] += f   # force (3) or force + moment (6)
        free = np.array([d for d in range(6 * n) if d // 6 not in self.fixed])
        Kff = K[free][:, free]
        # a vanishing spring on every free dof keeps a free spin (a member about its own axis) solvable; scaled by the median
        # diagonal: the mean let a stiff 'rigid' link put ~100 N/mm to ground on every node (27 Oct, frame v1 read low)
        Kff = Kff + csr_matrix((np.full(len(free), 1e-10 * np.median(Kff.diagonal())), (range(len(free)), range(len(free)))))
        u = np.zeros(6 * n); u[free] = spsolve(Kff, F[free])
        return u

    def modes(self, mass_g, n=6, rot_r=5.0):
        """lowest n natural frequencies (Hz) with lumped masses (g) at nodes: Guyan reduction onto the translational dofs of the
        nodes that carry mass (every other dof condensed out, massless), so a stiff / soft mix (carbon on TPU) stays well
        conditioned. Units t, mm, N. Returns (Hz, mode shapes on all dofs). rot_r is kept for the old call signature."""
        from scipy.linalg import eigh, solve
        nn = len(self.X); K = self.stiffness().toarray()
        free = [d for d in range(6 * nn) if d // 6 not in self.fixed]
        mst = [d for d in free if d % 6 < 3 and mass_g.get(d // 6, 0.0) >= 1.0]; slv = [d for d in free if d not in set(mst)]
        Kmm, Kms, Kss = K[np.ix_(mst, mst)], K[np.ix_(mst, slv)], K[np.ix_(slv, slv)]
        T = -solve(Kss, Kms.T, assume_a="sym"); Kr = Kmm + Kms @ T
        m = np.array([mass_g[d // 6] * 1e-6 for d in mst])
        w2, V = eigh((Kr + Kr.T) / 2, np.diag(m), subset_by_index=[0, min(n, len(mst)) - 1])
        U = np.zeros((6 * nn, len(w2))); U[mst] = V; U[slv] = T @ V
        return np.sqrt(np.maximum(w2, 0)) / (2 * np.pi), U

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
