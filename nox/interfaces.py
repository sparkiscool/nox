r"""interfaces: standard rule-sized interface features for Nox parts (preserve + obstacle bodies, P2 section 4).

Every function returns a Feature: an exact `preserve` solid (passive solid for printopt), an exact `obstacle` solid
(passive void: fastener, nut, rod, insertion and tool paths, already including every later cut), the P7 thickness
class, and `dims` (each dimension with the rule or hardware row it comes from). `Feature.check()` asserts the
feature's own rules (wall >= class floor, wall >= 1 D, outer size >= 3 D, edge distance >= 1.5 D, ...). `gate()` is the
P2 section 4 spec gate across several features, `voxelize()` turns preserve / obstacle bodies into the boolean
`solid` / `void` element masks of `tools\printopt.py` Problem (shape, h, origin, C-order ravel of (nx, ny, nz)).

Kernel: CadQuery 2.8 (OCP 7.9, pinned). Chosen because every existing Nox part script that draws these features
(scout_pocket.py, build_pocket10.py, cart12_spec.py, catcher fork_spec.py, scout_cage_nodes.py) is CadQuery, so the
solids drop straight into those scripts; build123d shares the same OCP kernel, so `Feature.preserve.wrapped` works there.

Local frame of every feature (then placed with at / axis / xdir):
  axis +z points OUT of the part, toward the side the screw head, mating part, rod or tool comes from;
  z = 0 is the interface face (head seat, mating face, nut-trap top face, socket mouth, pin base, contact face).
  xdir sets the in-plane direction (side nut slot exit, slot elongation, pad width).
Faces that lie on the part's outer surface are pushed PUSH = 0.5 outside it in the preserve body and obstacles reach
at least PUSH + OVERLAP = 0.8 past them (P2 section 8.4 / 8.5: trim once against the envelope; cuts overlap 0.3).
Tool / insertion paths default to TOOL_LEN = 40 mm, past the outside of any current Nox part: an obstacle may be longer
than needed (it is void outside the envelope anyway) but never shorter. Set tool_len to reach the outer surface if 40
is not enough.

Walls (radial, in the layer plane for a vertical axis) are snapped to the P7 section 4 line-width grid LINE_GRID;
axial thicknesses (roofs, floors, grip) are rounded up to whole layers (LAYER_H 0.16). The line-width table is E
(Bambu A1 mini default profile, Spark's profile unverified) and becomes BINDING ONLY AFTER COUPON K6 (P7 section 9);
until then the floors read as 0.8 / 1.2 / 1.6 / 2.5 and the grid is guidance.

Source tags (as in nox/dimensions.md): P measured, M mesh, C CAD / design doc, E estimate or unverified datasheet.
Units mm. Rules: R11 (3 Oct 2026).
"""
import math
from dataclasses import dataclass, field

import numpy as np
import cadquery as cq

# ---------------------------------------------------------------------------------------------------- constants
LINE_W = 0.42          # outer / default line width, Bambu A1 mini 0.4 nozzle system profile (P7 section 4, E)
LINE_STEP = 0.41       # each extra line adds about one line spacing (P7 section 4, E)
# P7 section 4 table (nominal wall for N lines). Other counts: 2 x LINE_W + (N - 2) x LINE_STEP.
# Binding only after coupon K6 confirms it on Spark's printer and profile.
LINE_GRID = {2: 0.84, 3: 1.25, 4: 1.66, 5: 2.07, 6: 2.47, 8: 3.28, 10: 4.10}
GRID_TOL = 0.05        # P7 section 4: draw walls at these values +/- 0.05
LAYER_H = 0.16         # 0.16mm Optimal @BBL A1M: Spark's active profile (P0; dry run gap 14). Was 0.2 (0.20mm
                       # Standard) until 3 Oct 2026. Axial thicknesses are counted from interface faces, not from the
                       # bed (that profile's first layer is 0.20, E). LINE_STEP / LINE_GRID stay P7 §4 (0.2-layer spacing
                       # formula; at 0.16 it gives ~0.416): a P7 change, confirmed by coupon K6, not changed here.
PUSH = 0.5             # P2 section 8.4: preserve faces on the envelope pushed 0.5 outside, trimmed once
OVERLAP = 0.3          # P2 section 8.5: cuts that meet overlap 0.3, never just touch
TOOL_LEN = 40.0        # default tool / insertion path length past the interface face
COS30 = math.cos(math.radians(30))


@dataclass(frozen=True)
class Cls:
    floor: float
    pref: float
    role: str
    src: str


# P7 section 3 (floors / preferred). T-b preferred is 2.0-2.5; 2.07 = 5 lines, the first grid value in that band.
CLASSES = {
    "P-c": Cls(1.66, 2.47, "PETG drawn load-bearing interface", "P7 s3 P-c"),
    "P-d": Cls(1.25, 1.66, "PETG drawn locating / skin", "P7 s3 P-d"),
    "P-e": Cls(0.84, 1.25, "PETG lip / lead-in / chamfer edge (backed)", "P7 s3 P-e"),
    "T-b": Cls(1.66, 2.07, "TPU socket sleeve", "P7 s3 T-b"),
    "T-d": Cls(1.66, 2.47, "TPU attachment (screw boss, press-fit peg)", "P7 s3 T-d"),
}


@dataclass(frozen=True)
class Dim:
    value: float
    src: str


# ------------------------------------------------------------------------------------------------ hardware data
# Screws: ISO 4762 socket head, ISO 10642 countersunk, ISO 4032 hex nut (standard tables, E until measured).
SCREWS = {
    "M2": dict(
        d=Dim(2.0, "ISO 261 nominal"),
        clr=Dim(2.4, "build_pocket10.py BOLT_D 2.4 (C, camera M2); = ISO 273 medium"),
        head_d=Dim(3.8, "ISO 4762 dk max (E)"),
        head_k=Dim(2.0, "ISO 4762 k (E)"),
        cb_d=Dim(4.3, "E: head 3.8 + 0.5, same margin as the M3 counterbore 6.0 on 5.5"),
        csk_d=Dim(4.4, "ISO 10642 dk theoretical (E); pack bar v2 uses M2x8 countersunk"),
        pilot=Dim(1.7, "scout_pocket.py M2_PILOT 1.7 / cart12_spec.py PILOT_R 0.85 (C, printed v11 / v12)"),
        nut_af=Dim(4.0, "ISO 4032 s; build_pocket10.py NUT_AF 4.0 (C)"),
        nut_m=Dim(1.6, "ISO 4032 m; build_pocket10.py NUT_T 1.6 (C)"),
    ),
    "M3": dict(
        d=Dim(3.0, "ISO 261 nominal"),
        clr=Dim(3.4, "scout_pocket.py / underside.py M3_CLR 3.4 (C, printed); nox-dimensions s1 'Pocket to airframe' 3 x M3"),
        head_d=Dim(5.5, "ISO 4762 dk max (E)"),
        head_k=Dim(3.0, "ISO 4762 k (E)"),
        cb_d=Dim(6.0, "scout_pocket.py CB_D 6.0 (C, printed pocket)"),
        csk_d=Dim(6.72, "ISO 10642 dk theoretical (E)"),
        pilot=Dim(2.5, "E: common M3 self-tap pilot in PETG, not yet printed on Nox"),
        nut_af=Dim(5.5, "nox-dimensions s2b 'Nut traps': M3 hex 5.5 AF (C); ISO 4032 s"),
        nut_m=Dim(2.4, "underside.py NUT_T 2.4 (C); ISO 4032 m"),
    ),
    # M5: swap-station frame screws (2040 legs, catcher flange). Standard table values from memory (E), nothing
    # printed on Nox yet; no self-tap pilot (threaded_hole) and no insert row for M5.
    "M5": dict(
        d=Dim(5.0, "ISO 261 nominal"),
        clr=Dim(5.5, "ISO 273 medium (E, not yet printed on Nox)"),
        head_d=Dim(8.5, "ISO 4762 dk max (E)"),
        head_k=Dim(5.0, "ISO 4762 k (E)"),
        cb_d=Dim(9.0, "E: head 8.5 + 0.5, same margin as the M3 counterbore 6.0 on 5.5"),
        csk_d=Dim(11.2, "ISO 10642 dk theoretical (E)"),
        nut_af=Dim(8.0, "ISO 4032 s (E)"),
        nut_m=Dim(4.7, "ISO 4032 m max (E)"),
    ),
}

# Washers under a screw head (clearance_hole(washer=...)): inner d1, outer d2, thickness h. Standard table values
# from memory, E until checked against the washers Spark has (P). DIN 9021 and ISO 7093-1 share d1 / d2 (ISO 7093-1
# starts at M3 and is 1.0 thick at M5; DIN 9021 1.2, used here: the taller head stack). DIN 125-1 A = ISO 7089.
WASHERS = {
    "DIN125": {   # normal washer, DIN 125-1 A / ISO 7089
        "M2": dict(d1=Dim(2.2, "DIN 125-1 A M2 d1 (E)"), d2=Dim(5.0, "DIN 125-1 A M2 d2 (E)"),
                   h=Dim(0.3, "DIN 125-1 A M2 h (E)")),
        "M3": dict(d1=Dim(3.2, "DIN 125-1 A M3 d1 (E)"), d2=Dim(7.0, "DIN 125-1 A M3 d2 (E)"),
                   h=Dim(0.5, "DIN 125-1 A M3 h (E)")),
        "M5": dict(d1=Dim(5.3, "DIN 125-1 A M5 d1 (E)"), d2=Dim(10.0, "DIN 125-1 A M5 d2 (E)"),
                   h=Dim(1.0, "DIN 125-1 A M5 h (E)")),
    },
    "DIN9021": {  # large washer, DIN 9021 / ISO 7093-1
        "M2": dict(d1=Dim(2.2, "DIN 9021 M2 d1 (E; not in ISO 7093-1)"), d2=Dim(6.0, "DIN 9021 M2 d2 (E)"),
                   h=Dim(0.5, "DIN 9021 M2 h (E)")),
        "M3": dict(d1=Dim(3.2, "DIN 9021 / ISO 7093-1 M3 d1 (E)"),
                   d2=Dim(9.0, "DIN 9021 / ISO 7093-1 M3 d2 (E; catcher spec 'DIN 9021 M3, OD 9')"),
                   h=Dim(0.8, "DIN 9021 / ISO 7093-1 M3 h (E)")),
        "M5": dict(d1=Dim(5.3, "DIN 9021 / ISO 7093-1 M5 d1 (E)"), d2=Dim(15.0, "DIN 9021 / ISO 7093-1 M5 d2 (E)"),
                   h=Dim(1.2, "DIN 9021 M5 h (E; ISO 7093-1 1.0)")),
    },
}
WASHER_STD = {"DIN125": "DIN125", "ISO7089": "DIN125", "normal": "DIN125",
              "DIN9021": "DIN9021", "ISO7093": "DIN9021", "large": "DIN9021"}
WASHER_EDGE = Dim(0.3, "E: radial margin round the washer footprint (placement + print tolerance)")
NUT_POCKET_CLR = Dim(0.3, "nox-dimensions s2b 'Nut traps': 5.5 AF + 0.3 from below (C)")
NUT_POCKET_EXTRA = Dim(0.7, "underside.py NUT_DEPTH 3.1 for a 2.4 nut (C): room for screw-length scatter")
NUT_SLOT_CLR_W = Dim(0.4, "build_pocket10.py side slot width NUT_AF + 0.4 (C)")
NUT_SLOT_CLR_T = Dim(0.3, "build_pocket10.py side slot height NUT_T + 0.3 (C)")

# Heat-set inserts: ruthex standard (CNC Kitchen pattern) catalogue values. Vendor data, [unverified] on Nox: E.
INSERTS = {
    "M2": dict(od=Dim(3.6, "ruthex RX-M2x4 OD (vendor, E)"), L=Dim(4.0, "ruthex RX-M2x4 length (vendor, E)"),
               hole=Dim(3.2, "ruthex recommended hole for M2 (vendor, E)")),
    "M3": dict(od=Dim(4.6, "ruthex RX-M3x5.7 OD (vendor, E)"), L=Dim(5.7, "ruthex RX-M3x5.7 length (vendor, E)"),
               hole=Dim(4.0, "ruthex recommended hole for M3 (vendor, E)")),
}

# Rods and bores.
RODS = {3.0: Dim(3.0, "nox-dimensions s3 'Rod': 3.0 solid pultruded carbon (C, not measured)"),
        4.0: Dim(4.0, "nox-dimensions s2 'Dowel pins' rod 4.0 (P); s4 Overseer catch rods <= 4 mm OD (C)")}
ROD_BORES = {  # (rod, material) -> bore
    (3.0, "PETG"): Dim(3.3, "nox-dimensions s3 'Socket bore, PETG with epoxy' 3.3 (P: slides in, firm)"),
    (3.0, "TPU95A"): Dim(3.3, "nox-dimensions s3 'Socket bore, TPU press fit' 3.3 provisional (CHECK until coupon G2/K2)"),
    (4.0, "PETG"): Dim(4.3, "E: 3 mm practice (+0.3) applied to 4 mm; 4.1 is the snug press (s2 'Dowel pins')"),
    (4.0, "TPU95A"): Dim(4.3, "E: 3 mm TPU practice (+0.3) applied to 4 mm; needs a bore coupon"),
}
ROD_LEADIN = Dim(0.6, "nox-dimensions s3 'Socket depth': lead-in 0.6 x 0.6 (C, v6 nodes)")
ROD_DEPTH_3 = Dim(8.0, "nox-dimensions s3 'Socket depth' 8 (C); pocket nodes 9 (s2 'Pocket cage nodes')")
PIN_PRESS = {4.0: Dim(4.1, "nox-dimensions s2 'Dowel pins': pocket holes 4.1 (snug) (C)")}
PIN_PROUD = Dim(14.0, "nox-dimensions s2 'Dowel pins': round-bore pin 14 proud (C)")
PIN_HOLE_DEPTH = Dim(6.0, "nox-dimensions s2 'Dowel pins': hole 6 deep (C)")
PIN_BORE = dict(  # cartridge v12 mating bore, nox-dimensions s2 'Cartridge pin bores' (C)
    band=Dim(4.4, "s2 'Cartridge pin bores' v12: 4.4 locating band (C)"),
    cone_d=Dim(8.0, "s2 'Cartridge pin bores' v12: mouth cone 8.0 (C)"),
    cone_len=Dim(3.5, "s2 'Cartridge pin bores' v12: cone over 3.5 (C)"),
    band_len=Dim(4.0, "s2 'Cartridge pin bores' v12: band z 3.5 to 7.5 (C)"),
    relief_d=Dim(6.5, "s2 'Cartridge pin bores' v12: 6.5 relief (C)"),
    depth=Dim(16.0, "s2 'Cartridge pin bores' v12: relief to z 16 (C)"),
    slot=Dim(0.8, "s2 'Cartridge pin bores': y 33 slot +/-0.4 (C)"),
)


# ----------------------------------------------------------------------------------------------- grid helpers
def wall_of(lines):
    """Nominal wall for a whole number of extrusion lines (P7 section 4)."""
    if lines < 2:
        raise ValueError("a drawn wall is at least 2 lines")
    return LINE_GRID.get(lines, round(2 * LINE_W + (lines - 2) * LINE_STEP, 2))


def lines_of(t):
    """Smallest line count whose nominal wall is >= t."""
    for n in range(2, 80):
        if wall_of(n) >= t - 0.005:
            return n
    raise ValueError(t)


def snap_wall(t):
    """Round a wall UP to the line-width grid."""
    return wall_of(lines_of(t))


def on_grid(t, tol=GRID_TOL):
    return any(abs(t - wall_of(n)) <= tol + 1e-9 for n in range(2, 80))


def snap_layers(t):
    """Round an axial thickness UP to whole layers."""
    return round(math.ceil(t / LAYER_H - 1e-6) * LAYER_H, 2)


# ------------------------------------------------------------------------------------------- geometry helpers
V = cq.Vector


def _cyl(r, z0, z1, slot=0.0):
    """Cylinder (or stadium elongated by `slot` along x) from z0 to z1 on the local z axis."""
    if slot > 0:
        return cq.Workplane("XY", origin=(0, 0, z0)).slot2D(slot + 2 * r, 2 * r).extrude(z1 - z0).val()
    return cq.Solid.makeCylinder(r, z1 - z0, V(0, 0, z0), V(0, 0, 1))


def _cone(r0, z0, r1, z1):
    return cq.Solid.makeCone(r0, r1, z1 - z0, V(0, 0, z0), V(0, 0, 1))


def _box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def _hex(e, z0, z1):
    """Hex prism, across corners e, corners on +-x (flats on +-y), from z0 to z1."""
    return cq.Workplane("XY", origin=(0, 0, z0)).polygon(6, e).extrude(z1 - z0).val()


def _revolve(pts):
    """Solid of revolution about local z from a closed (r, z) profile. Items: (r, z) or ('arc', (rm, zm), (r, z))."""
    wp = cq.Workplane("XZ").moveTo(*pts[0])
    for p in pts[1:]:
        wp = wp.threePointArc(p[1], p[2]) if p[0] == "arc" else wp.lineTo(*p)
    return wp.close().revolve(360, (0, 0, 0), (0, 1, 0)).val()


def _slotted_revolve(pts, slot):
    """As _revolve (straight segments only) but elongated by `slot` along x (the hull of two copies)."""
    s = _revolve(pts)
    if slot <= 0:
        return s
    half = [(r, z) for r, z in pts if r > 1e-9]
    poly = list(half) + [(-r, z) for r, z in reversed(half)]
    slab = cq.Workplane("YZ").polyline(poly).close().extrude(slot / 2, both=True).val()
    return _fuse(s.moved(cq.Location(V(-slot / 2, 0, 0))), s.moved(cq.Location(V(slot / 2, 0, 0))), slab)


def _fuse(*shapes):
    shapes = [s for s in shapes if s is not None]
    out = shapes[0]
    if len(shapes) > 1:
        out = out.fuse(*shapes[1:])
    return out.clean()


def _plane(at, axis, xdir):
    z = np.asarray(axis, float)
    z /= np.linalg.norm(z)
    ref = np.asarray(xdir, float) if xdir is not None else (np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0]))
    x = ref - (ref @ z) * z
    if np.linalg.norm(x) < 1e-9:
        raise ValueError("xdir parallel to axis")
    x /= np.linalg.norm(x)
    return cq.Plane(origin=tuple(float(a) for a in at), xDir=tuple(x), normal=tuple(z))


# ------------------------------------------------------------------------------------------------ the Feature
@dataclass
class Rule:
    item: str
    value: float
    op: str            # '>=' or '<='
    limit: float
    src: str
    level: str = "assert"   # 'assert' (FAIL stops) or 'warn'

    def ok(self):
        return self.value >= self.limit - 1e-6 if self.op == ">=" else self.value <= self.limit + 1e-6


@dataclass
class Feature:
    kind: str
    name: str
    preserve: object               # cq.Shape (placed)
    obstacle: object               # cq.Shape (placed)
    cls: str                       # P7 class of the drawn walls
    material: str
    dims: dict                     # name -> Dim
    rules: list                    # Rule
    at: tuple = (0.0, 0.0, 0.0)
    axis: tuple = (0.0, 0.0, 1.0)
    want_vertical: bool = False    # P2 s4: nut traps / inserts prefer a vertical axis
    edge_probe: object = None      # cylinder of radius 1.5 D over the clamped length, for edge_check()
    flags: list = field(default_factory=list)   # e.g. 'FLAGGED: side-entry nut trap', 'CHECK: provisional bore'
    notes: list = field(default_factory=list)

    def check(self, build_axis=None, strict=True):
        """Assert this feature's rules. Returns rows (dicts) for a table. build_axis = layer normal in part
        coordinates, e.g. (0, 0, 1); adds the axis-orientation warnings when given."""
        rows = []
        for r in self.rules:
            st = "ok" if r.ok() else ("FAIL" if r.level == "assert" else "WARN")
            rows.append(dict(feature=self.name, item=r.item, value=r.value, op=r.op, limit=r.limit, src=r.src, status=st))
        geo = [("preserve valid", self.preserve.isValid()), ("obstacle valid", self.obstacle.isValid()),
               ("preserve volume > 0", self.preserve.Volume() > 1e-3)]
        inter = self.preserve.intersect(self.obstacle).Volume()
        geo.append(("preserve x obstacle = 0 (P2 s4 gate)", inter < 1e-4))
        for item, ok in geo:
            rows.append(dict(feature=self.name, item=item, value=float(ok), op="==", limit=1.0, src="P2 s4",
                             status="ok" if ok else "FAIL"))
        if build_axis is not None:
            b = np.asarray(build_axis, float) / np.linalg.norm(build_axis)
            a = np.asarray(self.axis, float) / np.linalg.norm(self.axis)
            ang = math.degrees(math.acos(min(1.0, abs(float(a @ b)))))
            if self.want_vertical:
                rows.append(dict(feature=self.name, item="axis angle to build direction (deg)", value=ang, op="<=",
                                 limit=5.0, src="P2 s4: vertical axis preferred", status="ok" if ang <= 5 else "WARN"))
            if self.kind == "locating_pin" and self.dims["printed"].value and ang < 45:
                rows.append(dict(feature=self.name, item="printed pin bent across layers (axis angle deg)", value=ang,
                                 op=">=", limit=45, src="P2 s3: pins bent in-layer", status="WARN"))
        for f in self.flags:
            rows.append(dict(feature=self.name, item=f, value=float("nan"), op="", limit=float("nan"), src="", status="FLAG"))
        if strict:
            bad = [r for r in rows if r["status"] == "FAIL"]
            assert not bad, f"{self.name}: " + "; ".join(f"{r['item']} {r['value']:.3g} {r['op']} {r['limit']:.3g} ({r['src']})" for r in bad)
        return rows

    def edge_check(self, part_solid, tol=1e-3):
        """Edge distance >= 1.5 D in the real part: the 1.5 D cylinder around the axis must lie inside part_solid
        (the part's envelope or final solid); with a washer also its footprint pad under the seat (the washer must
        not hang over a side face). Returns the volume of the probe outside the part (0 = pass)."""
        if self.edge_probe is None:
            return 0.0
        out = self.edge_probe.cut(part_solid).Volume()
        what = "1.5 D zone / washer footprint" if "washer_d2" in self.dims else "1.5 D zone"
        assert out <= tol, f"{self.name}: edge distance < 1.5 D or washer off the seat, {out:.3f} mm3 of the {what} is outside the part"
        return out

    def export(self, folder, stem=None):
        """Write <stem>_preserve.stl and <stem>_obstacle.stl. Returns the two paths."""
        import os
        os.makedirs(folder, exist_ok=True)
        stem = stem or self.name
        out = []
        for tag, s in (("preserve", self.preserve), ("obstacle", self.obstacle)):
            p = os.path.join(folder, f"{stem}_{tag}.stl")
            cq.exporters.export(s, p, tolerance=0.01, angularTolerance=0.1)
            out.append(p)
        return out


def _finish(kind, name, pres, obs, cls, material, dims, rules, at, axis, xdir, **kw):
    """Cut the obstacle out of the preserve (they never overlap, P2 s4), place both, return the Feature."""
    pres = pres.cut(obs).clean()
    loc = cq.Location(_plane(at, axis, xdir))
    probe = kw.pop("edge_probe", None)
    return Feature(kind, name, pres.moved(loc), obs.moved(loc), cls, material, dims, rules, tuple(at), tuple(axis),
                   edge_probe=probe.moved(loc) if probe is not None else None, **kw)


def _cls_for(material, petg, tpu):
    if material not in ("PETG", "TPU95A"):
        raise ValueError("material is PETG (rigid parts) or TPU95A (cage nodes)")
    return petg if material == "PETG" else tpu


def _hole_rules(D, hole, wall, cls, d_src):
    """P2 s4 / P7 P-c rules for a hole of diameter `hole` with radial wall `wall` for a fastener of size D."""
    c = CLASSES[cls]
    outer = hole + 2 * wall
    return [
        Rule("radial wall >= class floor", wall, ">=", c.floor, c.src),
        Rule("radial wall >= 1 D", wall, ">=", D, "P2 s4 " + d_src),
        Rule("outer size >= 3 D", outer, ">=", 3 * D, "P2 s4"),
        Rule("edge distance (boss radius) >= 1.5 D", outer / 2, ">=", 1.5 * D, "P2 s4"),
        Rule("radial wall >= preferred", wall, ">=", c.pref, c.src + " preferred", "warn"),
        Rule("radial wall on line grid (+-0.05)", float(on_grid(wall)), ">=", 1.0, "P7 s4 (binding after K6)", "warn"),
    ]


def _head_path_d(S, head):
    if head == "socket":
        return S["cb_d"]
    if head == "csk":
        return Dim(S["csk_d"].value + 0.2, S["csk_d"].src + " + 0.2 clearance (E)")
    if head == "none":
        return S["clr"]
    raise ValueError("head is 'socket', 'csk' or 'none'")


def _default_wall(D, hole, cls, wall):
    if wall is not None:
        return wall
    return snap_wall(max(D, CLASSES[cls].pref, (3 * D - hole) / 2))


# ------------------------------------------------------------------------------------------------- features
def clearance_hole(size="M3", grip=None, head="socket", recess=0.0, slot=0.0, wall=None, material="PETG",
                   tool_len=TOOL_LEN, push=PUSH, washer=None, at=(0, 0, 0), axis=(0, 0, 1), xdir=None, name=None):
    """Clearance hole for an M2 / M3 / M5 screw that clamps this part (head on this part).
    Local: head seat at z = 0, clamped plastic z = -grip..0 (bottom face sits on the mating part), head side +z.
    recess > 0: counterbored head, seat at z = 0 and the boss continues up to z = recess (ring round the head).
    slot > 0: hole elongated along xdir by `slot` (pocket mounts use 2 x 2.0). head 'socket' | 'csk' | 'none'.
    washer: None | 'DIN125' (= 'ISO7089', 'normal') | 'DIN9021' (= 'ISO7093', 'large'): a washer under the head on
      the seat (not with 'csk'). Its footprint (d2 + the play of washer on shank and screw in the hole + WASHER_EDGE)
      sizes both bodies: the seat pad (preserve, class-floor thick, whole layers) is at least that wide, the obstacle
      above the seat (and a counterbore) is that wide, and edge_check() refuses a footprint that leaves the part, so
      a washer can never overlap a side face or a neighbouring wall.
    Preserve: boss of radial wall `wall` (default max(1 D, class preferred, 3 D outer), snapped to the line grid).
    Obstacle: shank through the grip and 0.8 past the bottom face, head (+ washer) + driver path up for tool_len."""
    S = SCREWS[size]
    D, clr = S["d"].value, S["clr"].value
    cls = _cls_for(material, "P-c", "T-d")
    hd = _head_path_d(S, head)
    if head == "csk" and (slot > 0 or recess > 0):
        raise ValueError("countersunk heads: no slot, no recess")
    W = None
    if washer is not None:
        if head == "csk":
            raise ValueError("countersunk heads take no washer")
        if washer not in WASHER_STD or size not in WASHERS[WASHER_STD[washer]]:
            raise ValueError(f"washer {washer!r} for {size}: one of {sorted(WASHER_STD)} for "
                             f"{sorted(WASHERS['DIN125'])}")
        W = WASHERS[WASHER_STD[washer]][size]
        # the washer slides on the shank (d1 - D) and the screw in the clearance hole (clr - D)
        r_w = W["d2"].value / 2 + (W["d1"].value - D) / 2 + (clr - D) / 2 + WASHER_EDGE.value
    csk_h = (hd.value - clr) / 2 if head == "csk" else 0.0
    grip = snap_layers(2 * D + csk_h) if grip is None else grip
    wall = _default_wall(D, clr, cls, wall)
    R = clr / 2 + wall
    r_path = max(hd.value / 2, r_w) if W else hd.value / 2           # head (+ washer) path above the seat
    w_head = snap_wall(CLASSES[cls].pref) if recess > 0 else 0.0
    R_top = max(R, r_path + w_head) if recess > 0 else R
    z_top = recess + push
    seat_t = snap_layers(CLASSES[cls].floor)
    R_seat = max(R, r_w) if W else R
    pres = _fuse(_cyl(R, -grip - push, z_top, slot), _cyl(R_top, 0.0, z_top, slot) if recess > 0 else None,
                 _cyl(R_seat, -min(seat_t, grip), z_top, slot) if W else None)
    t_up = max(tool_len, push + OVERLAP)
    obs = [_cyl(clr / 2, -grip - push - OVERLAP, 0.01, slot)]
    if head == "csk":
        obs += [_cone(clr / 2, -csk_h, hd.value / 2, 0.0), _cyl(hd.value / 2, -0.01, t_up)]
    else:
        obs.append(_cyl(r_path, 0.0, recess + t_up, slot))
    obs = _fuse(*obs)
    dims = dict(D=S["d"], clearance=S["clr"], head_path_d=hd, grip=Dim(grip, "E: 2 D (+ countersink) unless given"),
                wall=Dim(wall, "max(1 D, P7 preferred, 3 D outer), line grid"), outer=Dim(clr + 2 * wall, "derived"),
                recess=Dim(recess, "given"), slot=Dim(slot, "given"), tool_len=Dim(t_up, "P2 s4 tool path"))
    rules = _hole_rules(D, clr, wall, cls, "(radial wall >= max(1 D, 1.6))")
    rules.append(Rule("head / tool path reaches past the pushed face", t_up, ">=", push + OVERLAP, "P2 s8.4-8.5"))
    if head == "csk":
        rules.append(Rule("plastic under the countersink >= class floor", grip - csk_h, ">=", CLASSES[cls].floor, CLASSES[cls].src))
    if recess > 0:
        rules.append(Rule("wall round the counterbore >= class floor", R_top - r_path, ">=", CLASSES[cls].floor, CLASSES[cls].src))
    probe = _cyl(1.5 * D, -grip, 0.0, slot)
    if W:
        a_w = math.pi / 4 * (W["d2"].value ** 2 - clr ** 2)
        dims.update(washer=Dim(0.0, f"{WASHER_STD[washer]} {size}"), washer_d1=W["d1"], washer_d2=W["d2"],
                    washer_h=W["h"], washer_footprint_d=Dim(round(2 * r_w, 3), "d2 + play on shank and in the hole + "
                                                            f"2 x {WASHER_EDGE.value} edge (E)"),
                    seat_d=Dim(round(2 * R_seat, 3), "seat pad >= washer footprint"),
                    seat_t=Dim(min(seat_t, grip), CLASSES[cls].src + " floor, whole layers"),
                    washer_area=Dim(round(a_w, 2), "bearing area pi/4 (d2^2 - clearance^2), mm2"))
        rules.append(Rule("seat pad radius >= washer footprint radius", R_seat, ">=", r_w, "washer option (E sizes)"))
        if recess > 0:
            rules.append(Rule("recess >= head height + washer (flush head)", recess, ">=", S["head_k"].value +
                              W["h"].value, "ISO 4762 k + washer h (E)", "warn"))
        probe = _fuse(probe, _cyl(r_w, -min(seat_t, grip), 0.0, slot))
    nm = name or f"{size}_clearance_{head}" + (f"_{WASHER_STD[washer]}" if W else "")
    return _finish("clearance_hole", nm, pres, obs, cls, material, dims, rules, at, axis, xdir, edge_probe=probe)


def threaded_hole(size="M2", engage=None, tip_clear=1.0, floor=None, wall=None, material="PETG", mate_t=0.0,
                  chamfer=0.3, blind=True, tool_len=TOOL_LEN, push=PUSH, at=(0, 0, 0), axis=(0, 0, 1), xdir=None, name=None):
    """Screw driven straight into plastic (self-tapping into a printed pilot).
    Local: mating face at z = 0, screw enters from +z, pilot down to z = -(engage + tip_clear), blind floor below.
    mate_t: thickness of the clamped mating part above z = 0 (screw shank there, head + driver path above it).
    Preserve: boss (wall >= max(1 D, preferred), outer >= 3 D). Obstacle: pilot + entry chamfer + screw + head + tool."""
    S = SCREWS[size]
    if "pilot" not in S:
        raise ValueError(f"no self-tap pilot for {size} in SCREWS (not used on Nox yet); use a nut trap or an insert")
    D, pilot = S["d"].value, S["pilot"].value
    cls = _cls_for(material, "P-c", "T-d")
    c = CLASSES[cls]
    engage = snap_layers(2.5 * D) if engage is None else engage
    floor = snap_layers(c.floor) if floor is None else floor
    wall = _default_wall(D, pilot, cls, wall)
    R = pilot / 2 + wall
    depth = engage + tip_clear
    z_bot = -depth - (floor if blind else push)
    pres = _cyl(R, z_bot, push)
    t_up = max(tool_len, push + OVERLAP)
    obs = [_cyl(pilot / 2, -depth if blind else z_bot - OVERLAP, 0.01), _cone(pilot / 2, -chamfer, pilot / 2 + chamfer, 0.0)]
    if mate_t > 0:
        obs.append(_cyl(S["clr"].value / 2, -0.01, mate_t + 0.01))
    obs.append(_cyl(S["cb_d"].value / 2, mate_t, mate_t + t_up))
    obs = _fuse(*obs)
    dims = dict(D=S["d"], pilot=S["pilot"], engage=Dim(engage, "E: 2.5 D thread engagement in plastic"),
                tip_clear=Dim(tip_clear, "E"), floor=Dim(floor if blind else 0.0, c.src + " floor, whole layers"),
                wall=Dim(wall, "max(1 D, P7 preferred, 3 D outer), line grid"), outer=Dim(2 * R, "derived"),
                head_path_d=S["cb_d"])
    rules = _hole_rules(D, pilot, wall, cls, "(radial wall >= max(1 D, 1.6))")
    rules.append(Rule("thread engagement >= 2 D", engage, ">=", 2 * D, "E (no Nox rule; maker practice)", "warn"))
    if blind:
        rules.append(Rule("blind floor >= class floor", floor, ">=", c.floor, c.src))
    probe = _cyl(1.5 * D, -depth, 0.0)
    return _finish("threaded_hole", name or f"{size}_threaded", pres, obs, cls, material, dims, rules, at, axis, xdir,
                   edge_probe=probe)


def insert_boss(size="M3", wall=None, extra_depth=1.0, floor=None, material="PETG", mate_t=0.0, tool_len=TOOL_LEN,
                push=PUSH, at=(0, 0, 0), axis=(0, 0, 1), xdir=None, name=None):
    """Heat-set insert boss (ruthex-type insert, flush with the mating face).
    Local: mating face z = 0, insert hole down to z = -(L + extra_depth), blind floor below; iron and insert come
    from +z. Wall is measured from the insert hole; D-rules use the screw size D (see notes).
    Preserve: boss (default wall max(1 D, 6 lines): InsertGuide, pull-out ~2x from 3 to 6 perimeters).
    Obstacle: insert hole + iron / insert path (insert OD + 1.0) + screw clearance and head path above."""
    S, I = SCREWS[size], INSERTS[size]
    D, hole, L = S["d"].value, I["hole"].value, I["L"].value
    cls = _cls_for(material, "P-c", "T-d")
    c = CLASSES[cls]
    floor = snap_layers(c.floor) if floor is None else floor
    wall = _default_wall(D, hole, cls, wall)
    R = hole / 2 + wall
    depth = L + extra_depth
    pres = _cyl(R, -depth - floor, push)
    t_up = max(tool_len, push + OVERLAP)
    path_d = I["od"].value + 1.0
    obs = _fuse(_cyl(hole / 2, -depth, 0.01), _cyl(path_d / 2, 0.0, t_up),
                _cyl(S["cb_d"].value / 2, mate_t, mate_t + t_up) if mate_t > 0 else None)
    dims = dict(D=S["d"], insert_od=I["od"], insert_L=I["L"], hole=I["hole"],
                depth=Dim(depth, "E: insert length + 1.0 for melt and screw tip"),
                floor=Dim(floor, c.src + " floor, whole layers"), wall=Dim(wall, "max(1 D, 6 lines = 2.47), line grid"),
                outer=Dim(2 * R, "derived"), iron_path_d=Dim(path_d, "E: insert OD + 1.0"))
    rules = _hole_rules(D, hole, wall, cls, "(D = screw size)")
    rules += [Rule("hole depth >= insert length + 0.5", depth, ">=", L + 0.5, "E (vendor practice)"),
              Rule("blind floor >= class floor", floor, ">=", c.floor, c.src),
              Rule("wall >= 6 lines (insert pull-out)", wall, ">=", wall_of(6), "research report: InsertGuide 3 -> 6 perimeters", "warn")]
    probe = _cyl(1.5 * D, -depth, 0.0)
    return _finish("insert_boss", name or f"{size}_insert", pres, obs, cls, material, dims, rules, at, axis, xdir,
                   want_vertical=True, edge_probe=probe, notes=["insert sizes are vendor data (E): measure the inserts bought"])


def nut_trap(size="M3", entry="bottom", roof=None, wall=None, slot_len=None, tip_len=1.5, floor=None, mate_t=0.0,
             material="PETG", entry_len=TOOL_LEN, tool_len=TOOL_LEN, push=PUSH, at=(0, 0, 0), axis=(0, 0, 1),
             xdir=None, name=None):
    """Captive hex nut. The screw comes from +z through the mating part (thickness mate_t) and this part's roof.
    entry 'bottom' (preferred): hex pocket opening on the bottom face, nut pushed in from below.
        Local: top face z = 0, roof z = -roof..0, pocket (AF + 0.3, nut + 0.7 deep) below, bottom face open.
    entry 'side' (allowed, FLAGGED: research report, side pocket pulled out at ~86 kg vs 166 kg for a bottom pocket):
        slot of AF + 0.4 x nut + 0.3 swept along +xdir from the axis out to slot_len (axis to outer surface,
        default the boss radius), screw tip room tip_len and a floor below.
    Preserve: shell >= wall around the WHOLE nut envelope incl. the slot (wall measured from the hex corners),
    outer >= 3 D. Obstacle: nut, pocket / slot swept to the outer surface and past it (entry_len), screw, head, tool."""
    S = SCREWS[size]
    D, clr, af, m = S["d"].value, S["clr"].value, S["nut_af"].value, S["nut_m"].value
    cls = _cls_for(material, "P-c", "T-d")
    c = CLASSES[cls]
    roof = snap_layers(max(D, c.floor)) if roof is None else roof
    t_up = max(tool_len, push + OVERLAP)
    above = [_cyl(clr / 2, -roof - 0.01, mate_t + 0.01), _cyl(S["cb_d"].value / 2, mate_t, mate_t + t_up)]
    dims = dict(D=S["d"], nut_af=S["nut_af"], nut_m=S["nut_m"], roof=Dim(roof, "max(1 D, class floor), whole layers"))
    flags, notes = [], []
    if entry == "bottom":
        p_af = af + NUT_POCKET_CLR.value
        e = p_af / COS30
        p_h = m + NUT_POCKET_EXTRA.value
        wall = snap_wall(max(c.pref, (3 * D - e) / 2)) if wall is None else wall
        R = e / 2 + wall
        zb = -(roof + p_h)
        pres = _cyl(R, zb - push, push)
        obs = _fuse(_hex(e, zb - max(entry_len, push + OVERLAP), -roof), *above)
        dims.update(pocket_af=Dim(p_af, NUT_POCKET_CLR.src), pocket_depth=Dim(p_h, NUT_POCKET_EXTRA.src))
        probe = _cyl(1.5 * D, zb, 0.0)
        notes.append("print with the pocket opening on the bed or facing down; the roof bridges over the hex")
    elif entry == "side":
        s_w = af + NUT_SLOT_CLR_W.value
        s_h = m + NUT_SLOT_CLR_T.value
        e = s_w / COS30
        wall = snap_wall(max(c.pref, (3 * D - e) / 2)) if wall is None else wall
        R = e / 2 + wall
        floor = snap_layers(c.floor) if floor is None else floor
        z_nt, z_nb = -roof, -roof - s_h
        z_tip = z_nb - tip_len
        zb = z_tip - floor
        slot_len = R if slot_len is None else slot_len
        hw = s_w / 2 + wall
        pres = _fuse(_cyl(R, zb, push), _box(0.0, slot_len + push, -hw, hw, zb, push))
        x_out = slot_len + max(entry_len, push + OVERLAP)
        obs = _fuse(_hex(e, z_nb, z_nt), _box(0.0, x_out, -s_w / 2, s_w / 2, z_nb, z_nt), _cyl(clr / 2, z_tip, z_nb + 0.01),
                    *above)
        dims.update(slot_w=Dim(s_w, NUT_SLOT_CLR_W.src), slot_h=Dim(s_h, NUT_SLOT_CLR_T.src),
                    slot_len=Dim(slot_len, "axis to outer surface (default boss radius)"),
                    tip_len=Dim(tip_len, "E: screw tip past the nut"), floor=Dim(floor, c.src + " floor, whole layers"))
        flags.append("FLAGGED: side-entry nut trap (allowed; bottom entry or insert preferred, research report)")
        probe = _cyl(1.5 * D, zb, 0.0)
    else:
        raise ValueError("entry is 'bottom' or 'side'")
    dims.update(wall=Dim(wall, "shell round the nut corners: max(P7 preferred, 3 D outer), line grid"),
                outer=Dim(2 * R, "derived"))
    rules = [Rule("shell round the nut envelope >= class floor", wall, ">=", c.floor, c.src + "; P2 s4 nut trap >= 1.6"),
             Rule("shell >= preferred", wall, ">=", c.pref, c.src + " preferred", "warn"),
             Rule("shell on line grid (+-0.05)", float(on_grid(wall)), ">=", 1.0, "P7 s4 (binding after K6)", "warn"),
             Rule("outer size >= 3 D", 2 * R, ">=", 3 * D, "P2 s4"),
             Rule("edge distance (boss radius) >= 1.5 D", R, ">=", 1.5 * D, "P2 s4"),
             Rule("roof over the nut >= class floor", roof, ">=", c.floor, c.src),
             Rule("radial wall round the screw >= 1 D", R - clr / 2, ">=", D, "P2 s4")]
    if size == "M2":
        rules.append(Rule("M2 boss across >= 8", 2 * R, ">=", 8.0, "research report keep-zone table (nut ~4.6 across corners)"))
    if entry == "side":
        rules += [Rule("walls beside the slot >= class floor", wall, ">=", c.floor, c.src + " (camera mount broke at 0.6-0.7)"),
                  Rule("slot reaches the outer surface (slot_len >= boss radius)", slot_len, ">=", R, "P2 s4: slot swept out"),
                  Rule("floor under the screw tip >= class floor", floor, ">=", c.floor, c.src)]
    return _finish("nut_trap", name or f"{size}_nut_{entry}", pres, obs, cls, material, dims, rules, at, axis, xdir,
                   want_vertical=True, edge_probe=probe, flags=flags, notes=notes)


def locating_pin(D=4.0, proud=None, kind="rod", hole=None, hole_depth=None, collar_wall=None, fillet=None,
                 collar_depth=None, floor=None, mate_r=None, tip_chamfer=0.5, material="PETG", insert_len=TOOL_LEN,
                 at=(0, 0, 0), axis=(0, 0, 1), xdir=None, name=None):
    """Locating pin standing on the face z = 0, pointing +z, and the clear zone its mating part needs.
    kind 'rod': a cut rod pressed into a printed hole (pocket dowels: 4.0 rod, 4.1 hole, 6 deep, 14 proud).
        Preserve: root collar tube of radial wall collar_wall (>= 1 D) round the press hole, floor below.
    kind 'printed': a printed pin with a root fillet (>= max(1, D / 5.5)) onto the face and a collar (>= 1 D)
        under the face, collar_depth deep (default 1 D).
    Obstacle: the rod and its press hole (rod kind), plus the mating part's insertion zone: radius mate_r (default
    the mating bore band / 2 + 6 lines) from z = 0 up to proud + insert_len, minus the pin."""
    cls = _cls_for(material, "P-c", "T-d")
    c = CLASSES[cls]
    proud = PIN_PROUD.value if proud is None else proud
    collar_wall = snap_wall(max(D, c.pref)) if collar_wall is None else collar_wall
    rf = max(1.0, math.ceil(D / 5.5 * 10) / 10) if fillet is None else fillet
    mate_r = (D + 0.4) / 2 + wall_of(6) if mate_r is None else mate_r
    floor = snap_layers(c.floor) if floor is None else floor
    dims = dict(D=Dim(D, RODS.get(D, Dim(D, "given")).src), proud=Dim(proud, PIN_PROUD.src if proud == PIN_PROUD.value else "given"),
                collar_wall=Dim(collar_wall, "P2 s4 root collar >= 1 D, line grid"), mate_r=Dim(mate_r, "E: mating band / 2 + 6 lines"))
    rules = [Rule("root collar >= 1 D", collar_wall, ">=", D, "P2 s4"),
             Rule("root collar >= class floor", collar_wall, ">=", c.floor, c.src),
             Rule("insertion zone wider than the pin", mate_r, ">=", D / 2 + 0.5, "P2 s4 obstacle")]
    zone_top = proud + max(insert_len, OVERLAP)
    if kind == "rod":
        hole = PIN_PRESS.get(D, Dim(D + 0.1, "E: rod + 0.1 snug")).value if hole is None else hole
        hole_depth = PIN_HOLE_DEPTH.value if hole_depth is None else hole_depth
        Rc = hole / 2 + collar_wall
        pres = _cyl(Rc, -hole_depth - floor, 0.0)
        obs = _fuse(_cyl(hole / 2, -hole_depth, proud + 0.01), _cyl(mate_r, 0.0, zone_top))
        dims.update(hole=Dim(hole, PIN_PRESS.get(D, Dim(0, "E: rod + 0.1")).src), hole_depth=Dim(hole_depth, PIN_HOLE_DEPTH.src),
                    floor=Dim(floor, c.src + " floor, whole layers"), printed=Dim(0, "rod pin"))
        rules.append(Rule("floor under the press hole >= class floor", floor, ">=", c.floor, c.src))
        notes = ["root fillet >= 1 mm applies where generated material meets the collar (P2 s8.6)"]
    elif kind == "printed":
        rp = D / 2
        collar_depth = snap_layers(D) if collar_depth is None else collar_depth
        Rc = rp + collar_wall
        k = 1 - math.sqrt(0.5)
        pts = [(0, -collar_depth), (Rc, -collar_depth), (Rc, 0), (rp + rf, 0),
               ("arc", (rp + rf * k, rf * k), (rp, rf)), (rp, proud - tip_chamfer), (rp - tip_chamfer, proud), (0, proud)]
        pres = _revolve(pts)
        obs = _cyl(mate_r, 0.0, zone_top).cut(pres)
        dims.update(fillet=Dim(rf, "P2 s4 root fillet >= 1 mm (D / 5.5)"), collar_depth=Dim(collar_depth, "E: 1 D into the part"),
                    printed=Dim(1, "printed pin"))
        rules.append(Rule("root fillet >= max(1, D / 5.5)", rf, ">=", max(1.0, D / 5.5), "P2 s4; research report NASA D/5.5"))
        notes = ["printed pin: bending crosses the layers if the pin axis is the build direction (check() warns)"]
    else:
        raise ValueError("kind is 'rod' or 'printed'")
    return _finish("locating_pin", name or f"pin_{kind}_{D:g}", pres, obs, cls, material, dims, rules, at, axis, xdir,
                   notes=notes)


def locating_hole(D=4.0, band=None, band_len=None, cone_d=None, cone_len=None, relief_d=None, depth=None, slot=0.0,
                  wall=None, floor=None, pin_proud=None, material="PETG", entry_len=TOOL_LEN, push=PUSH,
                  at=(0, 0, 0), axis=(0, 0, 1), xdir=None, name=None):
    """Mating bore for a locating pin (default: cartridge v12 pin bore for the 4 mm dowels).
    Local: mouth face z = 0, bore runs to -z: lead-in cone cone_d -> band over cone_len, locating band band_len,
    45 deg step out to the relief relief_d down to -depth, blind floor below (floor=0: through).
    slot > 0: every section elongated along xdir (the slot-bore pin, +-0.4 -> slot 0.8).
    Preserve: tube of wall >= class preferred round the widest section below the cone. Obstacle: bore profile +
    the pin's entry path (cone mouth diameter) out of the mouth for entry_len."""
    cls = _cls_for(material, "P-c", "T-d")
    c = CLASSES[cls]
    P = PIN_BORE
    band = P["band"].value if band is None else band
    band_len = P["band_len"].value if band_len is None else band_len
    cone_d = P["cone_d"].value if cone_d is None else cone_d
    cone_len = P["cone_len"].value if cone_len is None else cone_len
    relief_d = P["relief_d"].value if relief_d is None else relief_d
    depth = P["depth"].value if depth is None else depth
    floor = snap_layers(c.floor) if floor is None else floor
    wall = snap_wall(c.pref) if wall is None else wall
    z1 = -cone_len
    z2 = z1 - band_len
    # the pin's entry path (cone mouth diameter, out of the mouth for entry_len) is the top of the SAME profile: a
    # separate entry prism fused on top left two coincident r = cone_d / 2 surfaces built two ways, and with a slot
    # OCC's clean() (UnifySameDomain) failed on them ("Courbes non jointives") at 22 cone diameters between 7.0 and
    # 8.8, e.g. 7.80 (Scout MANAGER, v13 prep, 3 Oct 2026). One revolve: no coincident pair, same solid.
    z_top = max(entry_len, push + OVERLAP)
    prof = [(0, z_top), (cone_d / 2, z_top), (cone_d / 2, 0.0), (band / 2, z1), (band / 2, z2)]
    if relief_d > band:
        prof.append((relief_d / 2, z2 - (relief_d - band) / 2))
    z_end = -depth if floor > 0 else -depth - push - OVERLAP
    prof += [(max(relief_d, band) / 2, z_end), (0, z_end)]
    bore = _slotted_revolve(prof, slot)
    R = max(band, relief_d) / 2 + wall
    pres = _cyl(R, -depth - (floor if floor > 0 else push), push, slot)
    obs = bore.clean()
    lip = R - cone_d / 2
    dims = dict(D=Dim(D, RODS.get(D, Dim(D, "given")).src), band=Dim(band, P["band"].src), cone_d=Dim(cone_d, P["cone_d"].src),
                cone_len=Dim(cone_len, P["cone_len"].src), band_len=Dim(band_len, P["band_len"].src),
                relief_d=Dim(relief_d, P["relief_d"].src), depth=Dim(depth, P["depth"].src), slot=Dim(slot, "given"),
                wall=Dim(wall, c.src + " preferred, line grid"), floor=Dim(floor, c.src + " floor, whole layers"),
                outer=Dim(2 * R, "derived"))
    rules = [Rule("wall round band / relief >= class floor", wall, ">=", c.floor, c.src),
             Rule("wall >= preferred", wall, ">=", c.pref, c.src + " preferred", "warn"),
             Rule("wall on line grid (+-0.05)", float(on_grid(wall)), ">=", 1.0, "P7 s4 (binding after K6)", "warn"),
             Rule("lip round the lead-in cone >= P-e floor", lip, ">=", CLASSES["P-e"].floor, "P7 s3 P-e (lead-in)"),
             Rule("band wider than the pin", band, ">=", D + 0.1, "fit")]
    if floor > 0:
        rules.append(Rule("blind floor >= class floor", floor, ">=", c.floor, c.src))
    if pin_proud is not None:
        rules.append(Rule("bore deeper than the pin is proud", depth, ">=", pin_proud + 0.5, "fit"))
    return _finish("locating_hole", name or f"pin_bore_{D:g}" + ("_slot" if slot else ""), pres, obs, cls, material,
                   dims, rules, at, axis, xdir)


def rod_socket(rod_d=3.0, material="PETG", bore=None, depth=None, wall=None, lead_in=None, floor=None, through=False,
               rod_len=TOOL_LEN, push=PUSH, at=(0, 0, 0), axis=(0, 0, 1), xdir=None, name=None):
    """Socket for a 3 mm (cage) or 4 mm (Overseer catch frame) carbon rod.
    PETG: epoxied slip fit, class P-c. TPU95A: press fit, class T-b (sleeve floor 1.66, preferred 2.0-2.5).
    Local: mouth face z = 0, rod comes out along +z; bore down to -depth (default 8 = nox-dimensions for 3 mm,
    2 D for 4 mm), lead-in lead_in x 45 deg at the mouth, blind floor below (through=True: open).
    Preserve: sleeve of wall `wall`. Obstacle: bore + lead-in + the rod and its insertion path out to rod_len."""
    if rod_d not in RODS:
        raise ValueError("rod_d is 3.0 or 4.0")
    cls = _cls_for(material, "P-c", "T-b")
    c = CLASSES[cls]
    bd = ROD_BORES[(rod_d, material)]
    bore = bd.value if bore is None else bore
    if depth is None:
        depth = ROD_DEPTH_3.value if rod_d == 3.0 else max(8.0, 2 * rod_d)
    lead_in = ROD_LEADIN.value if lead_in is None else lead_in
    floor = snap_layers(c.floor) if floor is None else floor
    wall = snap_wall(c.pref) if wall is None else wall
    R = bore / 2 + wall
    z_bot = -depth - (push if through else floor)
    pres = _cyl(R, z_bot, push)
    obs = _fuse(_cyl(bore / 2, -depth - (push + OVERLAP if through else 0.0), 0.01),
                _cone(bore / 2, -lead_in, bore / 2 + lead_in, 0.0),
                _cyl(bore / 2 + lead_in, 0.0, max(rod_len, push + OVERLAP)))
    flags = []
    if "CHECK" in bd.src or "E:" in bd.src:
        flags.append("CHECK: bore " + bd.src)
    dims = dict(rod=RODS[rod_d], bore=Dim(bore, bd.src if bore == bd.value else "given"),
                depth=Dim(depth, ROD_DEPTH_3.src if rod_d == 3.0 else "P2 s4: 2 x rod D"),
                lead_in=Dim(lead_in, ROD_LEADIN.src), wall=Dim(wall, c.src + " preferred, line grid"),
                floor=Dim(0.0 if through else floor, c.src + " floor, whole layers"), outer=Dim(2 * R, "derived"))
    rules = [Rule("sleeve wall >= class floor", wall, ">=", c.floor, c.src),
             Rule("sleeve wall >= preferred", wall, ">=", c.pref, c.src + " preferred", "warn"),
             Rule("sleeve wall on line grid (+-0.05)", float(on_grid(wall)), ">=", 1.0, "P7 s4 (binding after K6)", "warn"),
             Rule("depth >= 1.5 x rod D", depth, ">=", 1.5 * rod_d, "P2 s4"),
             Rule("depth >= 2 x rod D", depth, ">=", 2 * rod_d, "P2 s4 (upper end of 1.5-2 D)", "warn"),
             Rule("mouth wall after the lead-in >= P-e floor", wall - lead_in, ">=", CLASSES["P-e"].floor, "P7 s3 P-e (lead-in)")]
    if not through:
        rules.append(Rule("socket floor >= class floor", floor, ">=", c.floor, c.src))
    return _finish("rod_socket", name or f"rod{rod_d:g}_socket_{material}", pres, obs, cls, material, dims, rules,
                   at, axis, xdir, flags=flags)


def contact_pad(width, length, lines=None, cls="P-c", approach=TOOL_LEN, margin=0.0, push=PUSH,
                at=(0, 0, 0), axis=(0, 0, 1), xdir=None, name=None):
    """Flat contact face (seat, latch face, guide face, XT60 window backing) with N extrusion lines behind it.
    Local: contact face z = 0, normal +z toward the mating part; face width along xdir, length along y; material
    z = -t..0 with t = wall_of(lines) (default: class preferred; P2 s4 asks for >= 1.6 = 4 lines behind).
    Obstacle: the mating part and its approach path, the face footprint (+ margin) from z = 0 to `approach`.
    Note: 'lines behind' are perimeters only when the face normal lies in the layer plane; when the normal is the
    build direction the thickness is layers (round t up with snap_layers and pass lines accordingly)."""
    c = CLASSES[cls]
    lines = lines_of(c.pref) if lines is None else lines
    t = wall_of(lines)
    pres = _box(-width / 2, width / 2, -length / 2, length / 2, -t, push)
    obs = _box(-width / 2 - margin, width / 2 + margin, -length / 2 - margin, length / 2 + margin, 0.0,
               max(approach, push + OVERLAP))
    dims = dict(width=Dim(width, "given"), length=Dim(length, "given"), lines=Dim(lines, c.src + " preferred"),
                thickness=Dim(t, "P7 s4 line grid"), approach=Dim(approach, "mating part + approach path"))
    rules = [Rule("thickness behind the face >= class floor", t, ">=", c.floor, c.src),
             Rule("thickness >= preferred", t, ">=", c.pref, c.src + " preferred", "warn")]
    if cls == "P-c":
        rules.append(Rule("lines behind a load-bearing face >= 4", lines, ">=", 4, "P2 s4 (+1.6 behind); research report 3-4 perimeters"))
    return _finish("contact_pad", name or f"pad_{width:g}x{length:g}_{lines}L", pres, obs, cls,
                   "TPU95A" if cls.startswith("T") else "PETG", dims, rules, at, axis, xdir)


# ---------------------------------------------------------------------------------------- assemblies and gate
def union(shapes):
    shapes = [s for s in shapes if s is not None]
    return _fuse(*shapes) if shapes else None


def preserve_of(features):
    return union([f.preserve for f in features])


def obstacle_of(features):
    return union([f.obstacle for f in features])


def _bb_overlap(a, b):
    A, B = a.BoundingBox(), b.BoundingBox()
    return not (A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin)


def gate(features, tol=1e-3):
    """P2 s4 spec gate across features: no obstacle (later cut) may intersect any preserve body. Raises on failure.
    Use the same idea in a part spec for the part's own cuts: gate(features + [Feature-like cuts])."""
    bad = []
    for a in features:
        for b in features:
            if a is b or not _bb_overlap(a.obstacle, b.preserve):
                continue
            v = a.obstacle.intersect(b.preserve).Volume()
            if v > tol:
                bad.append(f"obstacle of {a.name} cuts preserve of {b.name}: {v:.3f} mm3")
    assert not bad, "P2 s4 gate failed:\n  " + "\n  ".join(bad)
    return True


def table(features, build_axis=None):
    """All check rows of several features (asserts each)."""
    rows = []
    for f in features:
        rows += f.check(build_axis=build_axis)
    return rows


# --------------------------------------------------------------------------------------------- voxelisation
def _grid(grid):
    if isinstance(grid, dict):
        return tuple(grid["shape"]), float(grid["h"]), tuple(grid["origin"])
    if hasattr(grid, "shape") and hasattr(grid, "h") and hasattr(grid, "origin"):
        return tuple(grid.shape), float(grid.h), tuple(grid.origin)
    shape, h, origin = grid
    return tuple(int(n) for n in shape), float(h), tuple(float(o) for o in origin)


def _triangles(obj, tol=0.01):
    """List of (n, 3, 3) triangle arrays, one per closed body (bodies are OR-ed later, never parity-mixed)."""
    if obj is None:
        return []
    if isinstance(obj, (list, tuple)):
        out = []
        for o in obj:
            out += _triangles(o, tol)
        return out
    if isinstance(obj, Feature):
        raise TypeError("pass feature.preserve / feature.obstacle, or preserve_of(features)")
    if isinstance(obj, cq.Workplane):
        return _triangles(list(obj.vals()), tol)
    if hasattr(obj, "vertices") and hasattr(obj, "faces") and not hasattr(obj, "tessellate"):   # trimesh
        return [np.asarray(obj.vertices, float)[np.asarray(obj.faces)]]
    out = []
    solids = obj.Solids() if hasattr(obj, "Solids") and obj.Solids() else [obj]
    for s in solids:
        vs, fs = s.tessellate(tol, 0.1)
        if not fs:
            continue
        P = np.array([(v.x, v.y, v.z) for v in vs], float)
        out.append(P[np.asarray(fs, int)])
    return out


def _inside(tris, xs, ys, zs):
    """Inside test on the lattice xs x ys x zs by z-scanline parity. tris (n, 3, 3). Returns bool (nx, ny, nz)."""
    nx_, ny_, nz_ = len(xs), len(ys), len(zs)
    dx = xs[1] - xs[0] if nx_ > 1 else 1.0
    dy = ys[1] - ys[0] if ny_ > 1 else 1.0
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    det = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
    keep = np.abs(det) > 1e-14
    a, b, c, det = a[keep], b[keep], c[keep], det[keep]
    lo, hi = np.minimum(np.minimum(a, b), c), np.maximum(np.maximum(a, b), c)
    i0 = np.clip(np.ceil((lo[:, 0] - xs[0]) / dx).astype(int), 0, nx_)
    i1 = np.clip(np.floor((hi[:, 0] - xs[0]) / dx).astype(int), -1, nx_ - 1)
    j0 = np.clip(np.ceil((lo[:, 1] - ys[0]) / dy).astype(int), 0, ny_)
    j1 = np.clip(np.floor((hi[:, 1] - ys[0]) / dy).astype(int), -1, ny_ - 1)
    ni, nj = np.maximum(i1 - i0 + 1, 0), np.maximum(j1 - j0 + 1, 0)
    cnt = ni * nj
    t = np.repeat(np.arange(len(cnt)), cnt)
    if len(t) == 0:
        return np.zeros((nx_, ny_, nz_), bool)
    k = np.arange(len(t)) - np.repeat(np.cumsum(cnt) - cnt, cnt)
    ii = i0[t] + k // nj[t]
    jj = j0[t] + k % nj[t]
    px, py = xs[ii], ys[jj]
    A, B, C, d = a[t], b[t], c[t], det[t]
    w1 = ((px - A[:, 0]) * (C[:, 1] - A[:, 1]) - (C[:, 0] - A[:, 0]) * (py - A[:, 1])) / d
    w2 = ((B[:, 0] - A[:, 0]) * (py - A[:, 1]) - (px - A[:, 0]) * (B[:, 1] - A[:, 1])) / d
    w0 = 1 - w1 - w2
    hit = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
    zh = (w0 * A[:, 2] + w1 * B[:, 2] + w2 * C[:, 2])[hit]
    col = (ii * ny_ + jj)[hit]
    z_base = min(zs[0], zh.min() if len(zh) else zs[0]) - 1.0
    span = max(zs[-1], zh.max() if len(zh) else zs[-1]) - z_base + 1.0
    key = np.sort(col * span + (zh - z_base))
    cols = np.arange(nx_ * ny_)
    start = np.searchsorted(key, cols * span)
    q = cols[:, None] * span + (zs[None, :] - z_base)
    below = np.searchsorted(key, q.ravel()).reshape(q.shape) - start[:, None]
    return (below % 2 == 1).reshape(nx_, ny_, nz_)


def fractions(body, grid, sub=3, tol=0.01):
    """Volume fraction of each element inside `body` (cq shape, list of shapes, Workplane or trimesh),
    estimated on sub^3 sample points per element. Returns a flat float array in printopt element order."""
    shape, h, origin = _grid(grid)
    nx, ny, nz = shape
    off = (np.arange(sub) + 0.5) / sub * h
    jit = np.array([0.000137, 0.000291, 0.000053]) * h   # no sample column runs exactly along a mesh edge
    xs = origin[0] + np.repeat(np.arange(nx) * h, sub) + np.tile(off, nx) + jit[0]
    ys = origin[1] + np.repeat(np.arange(ny) * h, sub) + np.tile(off, ny) + jit[1]
    zs = origin[2] + np.repeat(np.arange(nz) * h, sub) + np.tile(off, nz) + jit[2]
    ins = np.zeros((nx * sub, ny * sub, nz * sub), bool)
    for tr in _triangles(body, tol):
        ins |= _inside(tr, xs, ys, zs)
    return ins.reshape(nx, sub, ny, sub, nz, sub).mean(axis=(1, 3, 5)).ravel()


def voxelize(preserve, obstacle, grid, envelope=None, sub=3, frac=0.5, strict=True, info=False):
    """Preserve / obstacle bodies -> printopt passive masks (Problem.solid, Problem.void).
    grid: (shape, h, origin) | dict | printopt.Problem-like. Element e (C-order ravel of (nx, ny, nz), the order
    printopt uses; centre = index * h + origin + h / 2) is VOID if its obstacle fraction >= frac (or, with an
    envelope, its envelope fraction < frac), and SOLID if its preserve fraction >= frac and it is not void.
    Bodies may be shapes or lists of shapes (OR-ed). strict: raise if any element is more than frac inside both a
    preserve and an obstacle body (the P2 s4 gate failing at voxel level). Returns (solid, void) [, info dict]."""
    fp = fractions(preserve, grid, sub)
    fo = fractions(obstacle, grid, sub) if obstacle is not None else np.zeros_like(fp)
    void = fo >= frac
    if envelope is not None:
        void |= fractions(envelope, grid, sub) < frac
    conflict = (fp > frac) & (fo > frac)
    if strict and conflict.any():
        raise AssertionError(f"voxelize: {int(conflict.sum())} elements are inside both a preserve and an obstacle body")
    solid = (fp >= frac) & ~void
    if info:
        return solid, void, dict(conflict=int(conflict.sum()), f_preserve=fp, f_obstacle=fo)
    return solid, void
