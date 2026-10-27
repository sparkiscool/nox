"""Overseer core v2 picture: runs/overseer_core/v2/preview.png (python3 nox/overseer/core_v2_view.py after core_v2.py)."""
import json, trimesh, numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
d=json.load(open("nox/overseer/overseer_core.assembly.json")); P={p["name"]:p for p in d["parts"]}
L=lambda n: trimesh.load(P[n]["mesh"])
r=json.load(open("runs/overseer_core/v2/result.json"))
lt=np.array([0.3,-0.45,0.85]); lt/=np.linalg.norm(lt)
def draw(ax,m,col,a=1.0,maxf=20000):
    if len(m.faces)>maxf:
        idx=np.random.default_rng(0).choice(len(m.faces),maxf,replace=False); F=m.faces[idx]; N=m.face_normals[idx]
    else: F=m.faces; N=m.face_normals
    sh=0.42+0.58*np.clip(np.abs(N@lt),0,1); c=np.array(matplotlib.colors.to_rgb(col))[None]*sh[:,None]
    ax.add_collection3d(Poly3DCollection(m.vertices[F],facecolors=np.c_[c,np.full(len(c),a)],linewidths=0))
sl=r["slot"]; SX0,SX1=sl["x"]; SY=sl["y"]; SZ0,SZ1=sl["z"]
corn=np.array([[x,yy,z] for x in (SX0,SX1) for yy in (-SY,SY) for z in (SZ0,SZ1)])
edges=[(0,1),(2,3),(4,5),(6,7),(0,2),(1,3),(4,6),(5,7),(0,4),(1,5),(2,6),(3,7)]
groups=[("bottom plate","#c9ced6",0.35),("top plate","#c9ced6",0.18),("arm 1","#b8bec7",0.25),("arm 2","#b8bec7",0.25),("arm 3","#b8bec7",0.25),("arm 4","#b8bec7",0.25),
        ("FC stack (MicoAir743-Lite on 55A 4-in-1)","#3b4252",1),("Pi 5 + Active Cooler","#2e7d4f",0.9),("UBEC 5 V","#6b4fa0",1),("buck 12 V","#a04f8a",1),("RX XR2","#3b4252",1),("ESC cap","#3b4252",1),
        ("hub deck","#e8833a",1),("flow clip","#2f7fd1",1),("flow MTF-02P","#111111",1)]
keels=[np.array(v) for v in r["keels"].values()]
fig=plt.figure(figsize=(16,7.5),facecolor="white")
V=[(26,-58,"iso, nose right","full"),(0,-90,"side: deck on the plate, clip under it, slot below","full"),(90,-90,"plan (top plate hidden)","plan")]
for i,(el,az,t,mode) in enumerate(V):
    ax=fig.add_subplot(1,3,i+1,projection="3d")
    for n,c,a in groups:
        if mode=="plan" and n=="top plate": continue
        draw(ax,L(n),c,a)
    ax.add_collection3d(Line3DCollection([[corn[a],corn[b]] for a,b in edges],colors="#888",linewidths=1.2,linestyles="--"))
    for a,b in keels:
        e=a+(b-a)*0.32; ax.plot(*np.c_[a,e],color="#111",lw=3.2)
    ax.set_xlim(-130,135); ax.set_ylim(-110,110); ax.set_zlim(-80,75); ax.set_box_aspect((265,220,155)); ax.view_init(el,az); ax.set_axis_off(); ax.set_title(t,fontsize=12)
fig.suptitle("Overseer core v2 (system pass): orange hub deck = Pi + UBEC + buck + the keel front hub (2 sockets, black = the L2 head's keels, toward the nose), "
             "blue = flow clip under the front wing,\ndashed = the empty battery slot (latch later), dark = FC stack on the top plate. 2 printed pieces, 6 M3 screws up from below into inserts.",fontsize=11.5)
plt.tight_layout(rect=(0,0,1,0.9)); plt.savefig("runs/overseer_core/v2/preview.png",dpi=105)

# ---------------------------------------------------------------- alternatives (concept review 27 Oct: every alternative drawn)
from shapely.geometry import Polygon, Point, box as sbox
from shapely.ops import unary_union


def plan(m):
    tris = m.vertices[m.faces][:, :, :2]
    return unary_union([Polygon(t).buffer(0.01) for t in tris if Polygon(t).area > 1e-3])


def patch(ax, g, fc, ec="none", a=1.0, lw=0, ls="-"):
    """a shapely (multi)polygon with its holes, as one compound path"""
    from matplotlib.path import Path as MPath
    from matplotlib.patches import PathPatch
    for poly in getattr(g, "geoms", [g]):
        if poly.is_empty: continue
        verts, codes = [], []
        for ring in [poly.exterior] + list(poly.interiors):
            c = np.asarray(ring.coords)[:, :2]; verts += c.tolist(); codes += [MPath.MOVETO] + [MPath.LINETO] * (len(c) - 2) + [MPath.CLOSEPOLY]
        ax.add_patch(PathPatch(MPath(verts, codes), fc=fc, ec=ec, alpha=a, lw=lw, ls=ls))


deck2, clip2, plate2 = plan(L("hub deck")), plan(L("flow clip")), plan(L("bottom plate"))
slot2 = sbox(SX0, -SY, SX1, SY)                 # shapely box: minx, miny, maxx, maxy
feet = [tuple(f) for f in r["feet_xy"]]
fig2, axs = plt.subplots(1, 3, figsize=(16, 6), facecolor="white")
T_ = ["A (chosen): keel hub in the hub deck, flow clip on the 2 front screws\n2 pieces, 6 M3; front 4 feet alone carry <= 212 N a bolt (both keels push)",
      "B: separate front block for the keels on (96, ±17) + (60, ±17), deck as v1\n3 pieces, 8 M3; (60, ±17) heads land in the slot's reserved top",
      "C: flow on the deck, looking down through the plate\nno plate opening clears its 42° cone; under the plate the slot blocks it aft of x 91"]
for k, ax in enumerate(axs):
    patch(ax, plate2, "#d7dbe1"); patch(ax, slot2, "none", "#777", 1, 1.2, "--")
    if k in (0, 2): patch(ax, deck2, "#e8833a")
    if k == 1:
        patch(ax, deck2.difference(sbox(84, -30, 130, 30)), "#e8833a")
        patch(ax, unary_union([sbox(84, -22, 118, 22).intersection(deck2.buffer(4)), sbox(56, -20, 100, -14), sbox(56, 14, 100, 20), Point(60, 17).buffer(6), Point(60, -17).buffer(6)]), "#c0392b")
        for xy in [(60, 17), (60, -17), (96, 17), (96, -17)]: ax.plot(*xy, "kx", ms=7)
    if k == 0:
        patch(ax, clip2, "#2f7fd1", a=0.9)
        for xy in feet: ax.plot(*xy, "ko", ms=5)
    if k == 2:
        patch(ax, Point(10, -40).buffer(11), "#2f7fd1")
        patch(ax, Point(10, -40).buffer(29), "none", "#2f7fd1", 1, 1.2, ":")
    for a_, b_ in keels: ax.plot([a_[0], a_[0] + (b_ - a_)[0] * 0.12], [a_[1], a_[1] + (b_ - a_)[1] * 0.12], color="#111", lw=3)
    ax.set_xlim(-130, 140); ax.set_ylim(-75, 75); ax.set_aspect("equal"); ax.axis("off"); ax.set_title(T_[k], fontsize=10.5)
fig2.suptitle("Overseer core v2 alternatives, plan, nose right (grey = bottom plate, dashed = empty battery slot, black = keel stubs, dots = deck screws)", fontsize=12)
plt.tight_layout(rect=(0, 0, 1, 0.92)); plt.savefig("runs/overseer_core/v2/alternatives.png", dpi=100)
