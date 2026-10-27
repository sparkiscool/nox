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
sl=r["slot"]; x0,x1=sl["x"]; y=sl["y"]; z0,z1=sl["z"]
corn=np.array([[x,yy,z] for x in (x0,x1) for yy in (-y,y) for z in (z0,z1)])
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
