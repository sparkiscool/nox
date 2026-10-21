import json, math
d=json.load(open('s21_lantern12.json')); rods=d['rods']; nodes=[p for v in d['nodes'].values() for p in v]
hub=[(40*math.cos(a),40*math.sin(a),85) for a in [i*math.pi/4 for i in range(8)]]
def view(proj, W, Hh, x0, z0, extras):
    def P(p):
        p=(p+(0,))[:3] if len(p)<3 else p
        u,v=proj(p); return (u-x0, -(v-z0))
    out=[f'<svg viewBox="{-W/2:.0f} {-Hh/2:.0f} {W} {Hh}" width="100%" style="max-width:100%;display:block" font-family="IBM Plex Mono,monospace" font-size="7">']
    out.append(extras(P))
    for a,b,t in rods:
        (u1,v1),(u2,v2)=P(a),P(b)
        out.append(f'<line x1="{u1:.1f}" y1="{v1:.1f}" x2="{u2:.1f}" y2="{v2:.1f}" stroke="var(--carbon)" stroke-width="2" stroke-linecap="round"/>')
    for i in range(8):
        (u1,v1),(u2,v2)=P(hub[i]),P(hub[(i+1)%8])
        out.append(f'<line x1="{u1:.1f}" y1="{v1:.1f}" x2="{u2:.1f}" y2="{v2:.1f}" stroke="var(--node)" stroke-width="4"/>')
    PRINTED=[((x,y,33.2),(x,y,60)) for x,y in [(106,106),(-106,106),(-106,-106),(106,-106),(0,112),(-112,0),(0,-112)]]+[((112,14,24),(112,14,60)),((112,-14,24),(112,-14,60))]+[((37.5,s*32,-7),(37.5,s*32,-47)) for s in (1,-1)]
    for a,b in PRINTED:   # v12 printed columns and nose legs (were rods)
        (u1,v1),(u2,v2)=P(a),P(b)
        out.append(f'<line x1="{u1:.1f}" y1="{v1:.1f}" x2="{u2:.1f}" y2="{v2:.1f}" stroke="var(--node)" stroke-width="6" stroke-linecap="round"/>')
    for n in nodes:
        u,v=P(n); out.append(f'<circle cx="{u:.1f}" cy="{v:.1f}" r="3.2" fill="var(--node)"/>')
    out.append('</svg>'); return '\n'.join(out)
def rect(P,a,b,fill,op=.15,stroke=None):
    (u1,v1),(u2,v2)=P(a),P(b); x,y=min(u1,u2),min(v1,v2)
    return f'<rect x="{x:.1f}" y="{y:.1f}" width="{abs(u2-u1):.1f}" height="{abs(v2-v1):.1f}" fill="{fill}" fill-opacity="{op}"' + (f' stroke="{stroke}" stroke-width="1" stroke-dasharray="3 2"' if stroke else '') + '/>'
def line(P,a,b,stroke,dash='4 3',w=1.2):
    (u1,v1),(u2,v2)=P(a),P(b); return f'<line x1="{u1:.1f}" y1="{v1:.1f}" x2="{u2:.1f}" y2="{v2:.1f}" stroke="{stroke}" stroke-width="{w}" stroke-dasharray="{dash}"/>'
def text(P,p,s,anchor='start',fill='var(--muted)'):
    u,v=P(p); return f'<text x="{u:.1f}" y="{v:.1f}" text-anchor="{anchor}" fill="{fill}">{s}</text>'
# side: project (x,z) ; points given as (x,z) pairs -> pad
S=lambda p:(p[0],p[-1])
def side_extras(P):
    Q=lambda a:(a[0],0,a[1])
    s=[line(P,Q((84,-0.8)),Q((160,75.1)),'var(--muted)'), line(P,Q((-84,-0.8)),Q((-160,75.1)),'var(--muted)'), line(P,Q((-84,-0.8)),Q((84,-0.8)),'var(--muted)','2 3'),
       rect(P,Q((-98.55,37.2)),Q((98.55,45.2)),'var(--prop)',.12), rect(P,Q((42,-9.5)),Q((84,-60)),'var(--cat)',.18,'var(--cat)'),
       rect(P,Q((-47,-0.8)),Q((39,-60)),'var(--fc)',.10,'var(--fc)'), rect(P,Q((-41.5,-44.5)),Q((37.5,3.5)),'var(--pack)',.35), rect(P,Q((-50,18.5)),Q((50,22)),'var(--fg)',.5), rect(P,Q((99,22)),Q((100.6,60)),'var(--fc)',.6), rect(P,Q((100.6,35)),Q((114,47)),'var(--fc)',.6), text(P,Q((118,26)),'camera',fill='var(--fc)'),
       text(P,Q((88,-30)),'box arm sweep',fill='var(--cat)'), text(P,Q((-4,-56)),'swap head',anchor='middle',fill='var(--fc)'),
       text(P,Q((-168,80)),'funnel 45°'), text(P,Q((-104,42)),'props',anchor='end'),
       text(P,Q((122,88)),'z 85 hub'), text(P,Q((122,62)),'z 60 shoulder'), text(P,Q((122,35)),'z 33.2 skid ring'), text(P,Q((122,-3)),'z −0.8 opening'), text(P,Q((122,-46)),'z −47 sled'),
       text(P,Q((0,-72)),'side, nose to the right',anchor='middle',fill='var(--fg)')]
    return '\n'.join(s)
def front_extras(P):
    Q=lambda a:(0,-a[0],a[1])   # a=(y,z); we draw y to the right by projecting -y... keep simple: u = y
    Q=lambda a:(0,a[0],a[1])
    s=[line(P,Q((84,-0.8)),Q((160,75.1)),'var(--muted)'), line(P,Q((-84,-0.8)),Q((-160,75.1)),'var(--muted)'), line(P,Q((-84,-0.8)),Q((84,-0.8)),'var(--muted)','2 3'),
       rect(P,Q((9.65,37.2)),Q((98.55,45.2)),'var(--prop)',.12), rect(P,Q((-9.65,37.2)),Q((-98.55,45.2)),'var(--prop)',.12),
       rect(P,Q((-26,-0.8)),Q((14,-60)),'var(--fc)',.10,'var(--fc)'), rect(P,Q((-19,-44.5)),Q((19,3.5)),'var(--pack)',.35), rect(P,Q((-47,18.5)),Q((47,22)),'var(--fg)',.5),
       text(P,Q((-6,-56)),'swap head',anchor='middle',fill='var(--fc)'), text(P,Q((0,-72)),'front',anchor='middle',fill='var(--fg)'), text(P,Q((0,52)),'props',anchor='middle'),
       text(P,Q((122,-3)),'opening 168'), text(P,Q((122,-46)),'sled ±32')]
    return '\n'.join(s)
def top_extras(P):
    s=[]
    for mx,my in [(54.1,54.1),(-54.1,54.1),(54.1,-54.1),(-54.1,-54.1)]:
        u,v=P((mx,my,0)); s.append(f'<circle cx="{u:.1f}" cy="{v:.1f}" r="44.45" fill="var(--prop)" fill-opacity=".12" stroke="var(--prop)" stroke-width=".8"/>')
    s.append(rect(P,(-84,-84,0),(84,84,0),'var(--muted)',.0,'var(--muted)')); s.append(rect(P,(-41.5,-19,0),(37.5,19,0),'var(--pack)',.35))
    s.append(text(P,(0,-150,0),'top, nose to the right',anchor='middle',fill='var(--fg)')); s.append(text(P,(88,92,0),'opening 168'))
    return '\n'.join(s)
side=view(lambda p:(p[0],p[2]),390,186,0,18,side_extras)
front=view(lambda p:(p[1],p[2]),390,186,0,18,front_extras)
top=view(lambda p:(p[0],-p[1]),340,340,0,0,top_extras)
open('views12.html','w').write(f'<div class="views"><div>{side}</div><div>{front}</div><div class="topv">{top}</div></div>')
print('ok', len(side), len(front), len(top))
