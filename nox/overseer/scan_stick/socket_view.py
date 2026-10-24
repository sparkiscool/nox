import sys, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
S=sys.argv[1]; out=sys.argv[2]
m=trimesh.load(f"{S}/tt3.stl")
m.apply_transform(trimesh.transformations.rotation_matrix(np.pi,[1,0,0])); m.apply_translation([0,0,-m.bounds[0][2]])  # use pose: mouth at z 0
fig,axs=plt.subplots(1,2,figsize=(12,7.5),gridspec_kw=dict(width_ratios=[1.1,1]))
ax=axs[0]
sec=m.section(plane_origin=[0,0,0],plane_normal=[0,1,0])
P,_=sec.to_2D(to_2D=np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]]))
for poly in P.polygons_full:
    ax.add_patch(Polygon(np.array(poly.exterior.coords),closed=True,fc="#9db4c8",ec="#2b4a66",lw=1))
    for h in poly.interiors: ax.add_patch(Polygon(np.array(h.coords),closed=True,fc="white",ec="#2b4a66",lw=1))
# the pole (P Spark): Ø27.5 cylinder up to z 27, thread Ø19.05 x 25 to z 52
ax.add_patch(Rectangle((-13.75,-30),27.5,57,fc="#e8a23a",alpha=.55,ec="#9a5b00",lw=1.2))
ax.add_patch(Rectangle((-9.525,27),19.05,25,fc="#e8a23a",alpha=.35,ec="#9a5b00",lw=1.2,hatch="///"))
for z,t in ((0,"mouth"),(27,"shoulder seats, 27 sleeved"),(52,"tip, 0.5 under the top"),(52.5,"")):
    ax.axhline(z,color="#888",lw=.5,ls=":")
    if t: ax.text(19,z+0.6,t,fontsize=9)
ax.annotate("",xy=(-22,0),xytext=(-22,27),arrowprops=dict(arrowstyle="<->"))
ax.text(-27.5,13.5,"27",fontsize=10,rotation=90,va="center")
ax.annotate("",xy=(-22,27),xytext=(-22,52),arrowprops=dict(arrowstyle="<->"))
ax.text(-27.5,39.5,"25",fontsize=10,rotation=90,va="center")
ax.text(-13,-26,"pole Ø27.5",fontsize=9); ax.text(-8,44,"thread\n3/4-5 ACME",fontsize=8,ha="left")
ax.set_xlim(-30,42); ax.set_ylim(-31,58); ax.set_aspect("equal"); ax.set_title("fit coupon v3, cut through the axis, pole screwed in (orange)")
ax.set_xlabel("mm"); ax.grid(alpha=.2)
ax=axs[1]
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
axs[1].remove(); ax=fig.add_subplot(1,2,2,projection="3d")
q=m.copy(); q=q.slice_plane([0,0,0],[0,-1,0],cap=True)
ax.add_collection3d(Poly3DCollection(q.triangles,facecolors="#9db4c8",edgecolors="#9db4c8",linewidths=0.05,shade=True))
ax.set_xlim(-20,20); ax.set_ylim(-20,20); ax.set_zlim(0,53); ax.set_box_aspect((40,40,53)); ax.view_init(15,75); ax.set_axis_off()
ax.set_title("cutaway, mouth at the bottom (prints upside down)")
fig.suptitle("Scan stick socket: Ø28.3 sleeve 27 deep over the Ø27.5 cylinder, then the 26 thread (base v23 = the same cut)",fontsize=11)
fig.tight_layout(); fig.savefig(out,dpi=110)
