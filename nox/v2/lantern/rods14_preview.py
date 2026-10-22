"""Preview image: v13 vs v14 rod layouts, same view, props shaded. Writes runs/cage/v14-rods/preview.png"""
import json, math, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
d = json.load(open('rods14_export.json'))
INK, ROD, NEW, PRT, PROP = '#1d2430', '#5b6575', '#d0622c', '#2f6f8f', '#9fb4c8'
fig = plt.figure(figsize=(12, 6), dpi=150, facecolor='white')
for k, name in enumerate(('v13', 'v14')):
    ax = fig.add_subplot(1, 2, k+1, projection='3d'); ax.set_facecolor('white')
    old = {(tuple(a), tuple(b)) for a, b, t in d['v13']['rods']}
    for a, b, t in d[name]['rods']:
        col = NEW if name == 'v14' and (tuple(a), tuple(b)) not in old else ROD
        ax.plot(*zip(a, b), color=col, lw=2.0 if col == NEW else 1.4)
    for a, b, sz, t in d[name]['printed']: ax.plot(*zip(a, b), color=PRT, lw=4.0, solid_capstyle='round')
    th = np.linspace(0, 2*math.pi, 60)
    for mx, my in [(54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)]:
        ax.plot_trisurf(np.r_[mx, mx+44.45*np.cos(th)], np.r_[my, my+44.45*np.sin(th)], np.r_[41.2, 41.2+0*th], color=PROP, alpha=0.35, linewidth=0)
    ax.view_init(elev=24, azim=-58); ax.set_box_aspect((1, 1, 0.55)); ax.set_axis_off()
    ax.set_xlim(-130, 130); ax.set_ylim(-130, 130); ax.set_zlim(-55, 90)
    s = d[name]['score']
    ax.set_title(f"{name}   {s['mass_g']:.0f} g (E)   top hit {s['wall_J']:.2f} J   intrusion paths {s['probe_fail']}", color=INK, fontsize=11, pad=0)
fig.tight_layout(); fig.savefig('/home/claude/nox/runs/cage/v14-rods/preview.png', facecolor='white')
