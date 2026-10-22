"""S2 cage v2 strut route: straight 3.0 carbon struts from a bracket on the two side plate bolts to the 3 tee nodes.
Search bracket socket (y or x, z) for the lightest strut + hub arm that keeps 12 off the swept prop disks (z 37.2..45.2 since 17 Oct, prop plane 41.2,
r 44.45 at (+-54.1, +-54.1)), counting the socket bodies (r 2.9) on both ends."""
import numpy as np, itertools, json
PROP_Z0, PROP_Z1, DISK, MOT = 37.2, 45.2, 44.45, 54.1   # prop plane 41.2 +-4 sweep (D12 19.2, 17 Oct)
def d_disk(p):
    best = 1e9
    for sx, sy in itertools.product((1, -1), (1, -1)):
        c = np.array([sx * MOT, sy * MOT]); rp = np.linalg.norm(p[:2] - c)
        dr = max(0.0, rp - DISK); dz = max(0.0, PROP_Z0 - p[2], p[2] - PROP_Z1)
        best = min(best, np.hypot(dr, dz))
    return best
def clearance(a, b, r_end=2.9, r_rod=1.5, end_len=15.0):
    L = np.linalg.norm(b - a); worst = 1e9
    for t in np.linspace(0, 1, 300):
        p = a + (b - a) * t; r = r_end if min(t * L, (1 - t) * L) < end_len else r_rod
        worst = min(worst, d_disk(p) - r)
    return worst
R_ring, ZL = 112.0, 33.2   # lower ring = prop plane 41.2 - 8 (17 Oct)
def main():
  best = {}
  for name, axis in (("side", 1), ("tail", 0)):
      res = []
      for s_ in np.arange(30, 61, 1.0):
          if axis == 1 and s_ < 40: continue   # clear of the M3 side bolt at y 31 and its head (E)            # bracket socket bottom distance from centre along the axis
          for z in np.arange(15.0, 31.01, 0.25):
              if s_ < 50.0 and z < 25.0: continue   # over the plate: socket body (r 2.9) sits on the plate top z 22
              B = np.zeros(3); B[axis] = s_ if axis == 1 else -s_; B[2] = z
              R = np.zeros(3); R[axis] = R_ring if axis == 1 else -R_ring; R[2] = ZL
              if axis == 1:                          # 17 Oct flat print: the side hub prints upside down on its pad top
                  u = (R - B) / np.linalg.norm(R - B)  # (z 26, under the top case pad), so nothing of it may rise above
                  if z + 12.0 * u[2] + 2.9 * np.hypot(u[0], u[1]) > 26.0 + 1e-9: continue   # z 26 (socket 12 long, r 2.9)
              c = clearance(B, R)
              if c >= 12.0:
                  arm = np.linalg.norm(B[:2] - (np.array([3.9, 31.0]) if axis == 1 else np.array([3.9, 0.0])))
                  mass = np.linalg.norm(R - B) * 0.011 + arm * 0.032 + (25 - z) * 0.0   # 3.0 rod 11 g/m (E), PETG arm 5x5 (E)
                  res.append((mass, np.linalg.norm(R - B), s_, z, c, arm))
      res.sort(); mass, L, s_, z, c, arm = res[0]
      best[name] = dict(bracket_socket_bottom=[0, s_, z] if axis == 1 else [-s_, 0, z], ring_point=[0, R_ring, ZL] if axis == 1 else [-R_ring, 0, ZL],
                        centre_line_length=round(L, 1), prop_clearance=round(c, 2), bracket_arm=round(arm,1), mass_g_E=round(mass,2))
  print(json.dumps(best, indent=1))
  json.dump(best, open("strut_route.json", "w"))

if __name__ == '__main__':
    main()
