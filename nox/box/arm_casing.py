"""Box casing v1 (DEFAULT, Spark 15 Oct: printed, every piece on the 180 plate, glued): a 352 x 352 x 238 box of
4 mm PETG tiles joined with internal seam strips and corner angles. Faces (W coords, see arm_layout):
  stub wall  Y -4..0    partition between the wheel and the electronics bay; carries the stub flange (4 x M5 at 45 deg,
                        one bolt per tile). 4 identical tiles (rotated), generated ribs on the electronics side: TO run.
  far wall   Y 174..178 carries the bays (feet at r 150..172 on the ring). 2 x 2 tiles, drawn skin + rib grid (DEFAULT).
  back       Y -60..-56 closes the electronics bay. 2 x 2 plain ribbed tiles.
  rim        top (opening for the cartridge path + funnel feet), bottom, 2 sides: 176 x 119 ribbed tiles, 4 per face.
python arm_casing.py -> drawn tiles to out/, counts.  The stub tile's TO problem is in stub_tile_parts()."""
import os, math
import numpy as np
import manifold3d as mf
import arm_layout as L
from arm_cad import box, cylY, cylZ, cylX, hull, tm, report, OUT

T = L.CASE_T; TILE = L.TILE; RIB_H, RIB_T, RIB_P = 8.0, 2.0, 44.0


def ribbed_tile(w, h, rib_dirs=(0, 1), opening=None):
    """plain tile w x h in the XZ plane, skin Y 0..T, ribs toward +Y (inside), 2.4 x 10 on a 44 grid (DEFAULT), edge
    flange 10 x (T + 6) all round for the seam strips. opening = (x0, x1, z0, z1) cut through."""
    t = box(0, w, 0, T, 0, h)
    edge = box(0, w, T, T + 4, 0, h) - box(8, w - 8, T - 1, T + 5, 8, h - 8)
    t += edge
    n_x = int(w // RIB_P); n_z = int(h // RIB_P)
    for i in range(1, n_x):
        x = w * i / n_x; t += box(x - RIB_T / 2, x + RIB_T / 2, T, T + RIB_H, 0, h)
    for j in range(1, n_z):
        z = h * j / n_z; t += box(0, w, T, T + RIB_H, z - RIB_T / 2, z + RIB_T / 2)
    if opening is not None:
        x0, x1, z0, z1 = opening; t -= box(x0, x1, -1, T + RIB_H + 1, z0, z1)
        t += (box(x0 - 3, x1 + 3, T, T + 6, z0 - 3, z1 + 3) - box(x0, x1, T - 1, T + 7, z0, z1)) ^ box(0, w, 0, 20, 0, h)   # rim round the opening
    return t


def seam_strip(length):
    """glued over a seam on the inside: 24 x 2.4, with a 2.4 x 6 key along the middle that sits between the tiles' flanges"""
    return box(-12, 12, 0, 2.4, 0, length)


def corner_angle(length):
    return box(0, 20, 0, 2.4, 0, length) + box(0, 2.4, 0, 20, 0, length)


def far_wall_tile(ix, iz):
    """far wall at Y 174..178 (inner face Y 174, ribs toward -Y into the wheel chamber: outside the bay ring only,
    r > 172 is nothing, so ribs sit at r < 150 between bays... DEFAULT: ribs only outboard of r 150 are impossible; the
    far wall gets its stiffness from the 3 x 3 bay feet bosses + the edge flanges; ribs toward +Y (outside) instead."""
    x0 = -TILE + ix * TILE; z0 = -TILE + iz * TILE
    t = ribbed_tile(TILE, TILE)                                   # skin Y 0..4, ribs +Y (outside)
    t = t.translate([x0, L.Y_FAR_WALL, z0])
    # bay feet bosses: inside, at each bay's 3 foot positions (bay_parts feet), r 150..172
    bosses = None
    xc0, xc1 = -L.Z_TABC, L.CART_L - L.Z_TABC
    for a in list(L.SCOUT_BAYS) + list(L.OVERSEER_BAYS):
        for (x, r) in ((0.0, L.FOOT_R[0]), (0.0, L.FOOT_R[1])):          # v4: 2 feet per bay module (Overseer modules: same pattern, DEFAULT)
            # foot at (x, r) in the bay's own frame at the top; rotate to angle a about Y: top frame +Z radial
            th = math.radians(90 - a); X = x * math.cos(th) + r * math.sin(th); Z = -x * math.sin(th) + r * math.cos(th)
            if x0 <= X < x0 + TILE and z0 <= Z < z0 + TILE:
                b = cylY(X, Z, L.Y_FAR_WALL, L.Y_FAR_WALL + T + 4.0, 6.0) - cylY(X, Z, L.Y_FAR_WALL - 1, L.Y_FAR_WALL + T + 5, 1.7)   # M3 from outside into the module's heat-set insert
                bosses = b if bosses is None else bosses + b
    return (t + bosses) ^ box(x0, x0 + TILE, L.Y_FAR_WALL - 5, L.Y_FAR_WALL + 20, z0, z0 + TILE) if bosses is not None else t


def stub_tile_parts():
    """stub wall NE tile (X 0..176, Z 0..176 of the wall; the flange centre at the tile's (0, 0) corner) at Y -4..0.
    Drawn: skin 4, flange seat ring (quarter of Ø90 + 4 bolt boss), edge flanges on the two outer edges, seam flanges
    on the two inner edges; generated: ribs toward -Y (electronics bay) up to 16 deep, outside the PSU footprint.
    The other 3 tiles are this one rotated about the wheel axis."""
    y0, y1 = L.Y_STUB_WALL
    skin = box(0, TILE, y0 + 1.0, y1, 0, TILE)                                                # 3 mm skin
    seat = cylY(0, 0, y0 - 2.0, y0, 46.0) ^ box(0, 50, y0 - 3, y1, 0, 50)                     # 2 mm seat pad, quarter disc
    bx, bz = L.part_angle_pos(38.0, 45)
    boss = cylY(bx, bz, y0 - 6.0, y1, 6.0)
    edges = (box(TILE - 8, TILE, y0 - 4.0, y1, 0, TILE) + box(0, TILE, y0 - 4.0, y1, TILE - 8, TILE)) + \
            (box(0, 8, y0 - 4.0, y1, 0, TILE) + box(0, TILE, y0 - 4.0, y1, 0, 8))
    drawn = skin + seat + boss + edges
    cuts = cylY(bx, bz, y0 - 7, y1 + 1, 2.7) + cylY(0, 0, y0 - 3, y1 + 1, 12.5)             # M5 bolt; Ø25 lead hole (quarter)
    cuts += box(0, 120, y0 - 40, y0 - 16.0, 0, 176) - box(-1, 121, y0 - 41, y0 - 17, -1, 177) # (nothing: ribs max 16)
    env = box(0, TILE, y0 - 16.0, y1, 0, TILE)
    env -= box(40, 150, y0 - 17, y0 - 4.0, 60, 176)                                          # PSU footprint keep-out (E): ribs 0 there
    env = env + drawn
    return dict(drawn=drawn, cuts=cuts, env=env, allowed=env - cuts)


def all_drawn_tiles():
    parts = {}
    RIM = (L.Y_FAR_OUT - L.Y_BACK_OUT) / 2.0                                                 # 124 (box Y -60..188): 2 tiles per face along Y
    parts["case_rim_tile_v1"] = ribbed_tile(TILE, RIM)                                      # 14 of these (top has 2 special)
    parts["case_back_tile_v1"] = ribbed_tile(TILE, TILE)                                    # 4
    x0, x1, yy0, yy1 = L.TOP_OPENING
    # top rim: the two tiles over the drone position carry the opening; tile local: X along the box X, Z along Y
    # top tiles span X -176..0 and 0..176, each 176 x 119 along Y (-60..59, 59..178). Opening Y 92..152 is in the
    # +Y tiles only: X -45..50 crosses the X = 0 seam -> two special tiles
    for name, tx0 in (("case_top_tile_L_v1", -176.0), ("case_top_tile_R_v1", 0.0)):
        ox0, ox1 = max(x0, tx0) - tx0, min(x1, tx0 + 176) - tx0
        ymid = L.Y_BACK_OUT + RIM                                                            # the +Y pair starts here (64)
        parts[name] = ribbed_tile(TILE, RIM, opening=(ox0, ox1, yy0 - ymid, yy1 - ymid))
    for ix in range(2):
        for iz in range(2): parts[f"case_far_tile_{ix}{iz}_v1"] = far_wall_tile(ix, iz)
    parts["case_seam_strip_176_v1"] = seam_strip(176.0)
    parts["case_corner_angle_176_v1"] = corner_angle(176.0)
    return parts


if __name__ == "__main__":
    P = all_drawn_tiles()
    for k, v in P.items():
        report(k, v); tm(v).export(os.path.join(OUT, k + ".stl"))
    S = stub_tile_parts(); report("stub tile drawn", S["drawn"]); report("stub tile env", S["env"])
    print("tiles to print: rim 14 + top 2 + back 4 + far 4 + stub 4 = 28; seam strips ~24; corner angles 12")
