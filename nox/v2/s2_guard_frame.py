import sys,json,itertools,time
"""S2 guard frame model + load cases + rod-size search (guard chat, 12 Oct 2026).
Run: python3 s2_guard_frame.py  -> checks the chosen design (DEFAULT 23:55).
Bundled from fe.py, base.py, cases.py, search.py of the cloud session. Numbers tagged C (model) unless noted.
13 Oct (slider chat): strut section chosen by its tee (a tail strut off the centre line keeps strut_tail), one TPU link
per tee; slider_sockets_13oct() checks the sockets moved above the plate face. The chosen design's numbers are unchanged.
13 Oct (seat chat): leg tip height is params(tip_z=...) (rod end; default -26.5), tip_nodes(g) finds the leg ends;
used by v2/box_seat_checks.py. 13 Oct (guard chat): restored after an overwrite; the variation sweep model is
v2/s2_guard_sweep.py."""
# ---------------- fe.py ----------------
"""S2 guard frame model (C). Units N, mm, MPa, tonne-free: mass in g reported separately.
Carbon rods: E 125000 (E), G 5000 (E), rho 1.55 g/cc. TPU for AMS nodes: E 1190 (datasheet XY, CHECK), rho 1.22.
Struts clamped at the slider sockets. Joints at nodes rigid (node posts modelled as TPU beams)."""
import numpy as np
M_=54.1; RD=44.5; SW=4.0
MOTORS=[(sx*M_,sy*M_) for sx in(1,-1) for sy in(1,-1)]
EC,GC,EC_RHO=125000.,5000.,1.55e-3
ET,GT,ET_RHO=1190.,400.,1.22e-3

def sec(s):
    """s = ('solid',d) or ('tube',do,di)"""
    if s[0]=='solid': d=s[1]; A=np.pi*d*d/4; I=np.pi*d**4/64; c=d/2
    else: do,di=s[1],s[2]; A=np.pi*(do*do-di*di)/4; I=np.pi*(do**4-di**4)/64; c=do/2
    return dict(A=A,I=I,J=2*I,c=c,d=(s[1]))

def frame_k(p1,p2,E,G,A,I,J):
    L=np.linalg.norm(p2-p1); ex=(p2-p1)/L
    ref=np.array([0,0,1.]) if abs(ex[2])<0.95 else np.array([1.,0,0])
    ey=np.cross(ref,ex); ey/=np.linalg.norm(ey); ez=np.cross(ex,ey)
    R=np.vstack([ex,ey,ez]); T=np.zeros((12,12))
    for i in range(4): T[3*i:3*i+3,3*i:3*i+3]=R
    k=np.zeros((12,12)); a=E*A/L; t=G*J/L
    b1=12*E*I/L**3; b2=6*E*I/L**2; b3=4*E*I/L; b4=2*E*I/L
    k[0,0]=k[6,6]=a; k[0,6]=k[6,0]=-a
    k[3,3]=k[9,9]=t; k[3,9]=k[9,3]=-t
    for (v,r,s) in ((1,5,1),(2,4,-1)):
        k[v,v]=k[v+6,v+6]=b1; k[v,v+6]=k[v+6,v]=-b1
        k[v,r]=k[r,v]=s*b2; k[v,r+6]=k[r+6,v]=s*b2
        k[v+6,r]=k[r,v+6]=-s*b2; k[v+6,r+6]=k[r+6,v+6]=-s*b2
        k[r,r]=k[r+6,r+6]=b3; k[r,r+6]=k[r+6,r]=b4
    return T.T@k@T, T, L

class Guard:
    def __init__(s,P):
        s.P=P; s.nodes=[]; s.elems=[]; s.fixed=[]
        s.build()
    def node(s,p):
        p=np.asarray(p,float)
        for i,q in enumerate(s.nodes):
            if np.linalg.norm(q-p)<1e-6: return i
        s.nodes.append(p); return len(s.nodes)-1
    def line(s,a,b,secd,mat,tag,n=None,ln=10.):
        a=np.asarray(a,float); b=np.asarray(b,float); L=np.linalg.norm(b-a)
        n=n or max(2,int(np.ceil(L/ln)))
        ids=[s.node(a+(b-a)*i/n) for i in range(n+1)]
        for i in range(n): s.elems.append(dict(n=(ids[i],ids[i+1]),sec=secd,mat=mat,tag=tag))
        return ids
    def build(s):
        P=s.P; F,C=P['F'],P['C']; ZL,ZU=P['ZL'],P['ZU']
        oct8=[(F,-C),(F,C),(C,F),(-C,F),(-F,C),(-F,-C),(-C,-F),(C,-F)]
        tees=P['tees']; extra=P.get('extra',[])
        pts=[]
        for i in range(8):
            a=np.array(oct8[i]);b=np.array(oct8[(i+1)%8]);pts.append(('c',tuple(a)))
            seg=[]
            for kind,lst in (('t',tees),('x',extra)):
                for t in lst:
                    t=np.array(t,float);ab=b-a;u=np.dot(t-a,ab)/np.dot(ab,ab)
                    if 0<u<1 and np.linalg.norm(a+u*ab-t)<1e-6: seg.append((u,kind,tuple(t)))
            for u,kind,t in sorted(seg): pts.append((kind,t))
        s.ringpts=pts
        Ls,Us=sec(P['lower']),sec(P['upper'])
        for z,sc,tag in ((ZL,Ls,'lower'),(ZU,Us,'upper')):
            for i in range(len(pts)):
                a=(*pts[i][1],z); b=(*pts[(i+1)%len(pts)][1],z)
                s.line(a,b,sc,'C',tag)
        # node posts (TPU) between rings at every ring point
        for kind,(x,y) in pts:
            if kind=='x': continue
            d=7.5 if kind=='c' else 8.8
            if kind=='t' and not P.get('upper_at_tees',True):
                s.line((x,y,ZL),(x,y,4.0),sec(('solid',d)),'T','post',n=3)
            else:
                s.line((x,y,ZL),(x,y,ZU),sec(('solid',d)),'T','post',n=4)
        # struts: socket (clamped) -> attach point -> short TPU link to the tee post
        links_done=set()
        for (sock,att,post) in P['struts']:
            st=P['strut'] if (att[1]!=0 or not P.get('strut_tail')) else P['strut_tail']   # (slider chat: section by tee, not by socket y)
            ids=s.line(sock,att,sec(st),'C','strut')
            s.fixed.append(ids[0])
            if (tuple(att),tuple(post)) not in links_done:
                s.line(att,post,sec(('solid',8.0)),'T','link',n=1); links_done.add((tuple(att),tuple(post)))
        # legs
        if P.get('legs'):
            Lg=sec(P['leg'])
            for (corner,top,tip) in P['legs']:
                s.line(corner,top,sec(('solid',7.0)),'T','link',n=1)
                s.line(top,tip,Lg,'C','leg',n=3)
        s.N=len(s.nodes); s.X=np.array(s.nodes)
        s.assemble()
    def assemble(s):
        n=6*s.N; K=np.zeros((n,n))
        for e in s.elems:
            E,G=(EC,GC) if e['mat']=='C' else (ET,GT)
            p1,p2=s.X[e['n'][0]],s.X[e['n'][1]]
            k,T,L=frame_k(p1,p2,E,G,e['sec']['A'],e['sec']['I'],e['sec']['J'])
            e['T']=T; e['L']=L; e['E']=E; e['G']=G; e['kl']=T@k@T.T
            dof=np.r_[6*e['n'][0]:6*e['n'][0]+6,6*e['n'][1]:6*e['n'][1]+6]
            K[np.ix_(dof,dof)]+=k
        s.K=K
        fx=np.zeros(n,bool)
        for i in s.fixed: fx[6*i:6*i+6]=True
        s.free=~fx
        Kf=K[np.ix_(s.free,s.free)]
        s.Kinv=np.linalg.inv(Kf)
    def solve(s,F):
        u=np.zeros(6*s.N); u[s.free]=s.Kinv@F[s.free]; return u
    def flex(s,ids,dirs):
        """flexibility matrix between point loads (node ids, unit dirs)"""
        m=len(ids); Cm=np.zeros((m,m)); cols=[]
        for j,(i,d) in enumerate(zip(ids,dirs)):
            F=np.zeros(6*s.N); F[6*i:6*i+3]=d; u=s.solve(F); cols.append(u)
        for a,(i,d) in enumerate(zip(ids,dirs)):
            for j in range(m): Cm[a,j]=np.dot(cols[j][6*i:6*i+3],d)
        return Cm,cols
    def rigid_indent(s,ids,dirs,energy):
        """rigid flat indenter (wall/pole/floor) pushing along dirs at contact ids; returns forces, u, total F, delta"""
        act=list(range(len(ids)))
        Cm,cols=s.flex(ids,dirs)
        while True:
            Ca=Cm[np.ix_(act,act)]
            f=np.linalg.solve(Ca,np.ones(len(act)))  # forces for unit common displacement
            if (f>=-1e-9).all(): break
            act=[a for a,v in zip(act,f) if v>1e-9]
        k=f.sum()                      # N/mm
        delta=np.sqrt(2*energy/k)
        fa=f*delta
        u=sum(fa[q]*cols[a] for q,a in enumerate(act))
        return dict(k=k,delta=delta,F=k*delta,u=u,forces={ids[a]:fa[q] for q,a in enumerate(act)})
    def stresses(s,u):
        out=[]
        for e in s.elems:
            dof=np.r_[6*e['n'][0]:6*e['n'][0]+6,6*e['n'][1]:6*e['n'][1]+6]
            fl=e['kl']@(e['T']@u[dof])  # local end forces
            sg=0
            for end in (0,6):
                N=fl[end]; My,Mz=fl[end+4],fl[end+5]
                M=np.hypot(My,Mz); sg=max(sg,abs(N)/e['sec']['A']+M*e['sec']['c']/e['sec']['I'])
            e_end=[np.hypot(fl[4],fl[5]),np.hypot(fl[10],fl[11])]
            out.append((sg,e_end))
        return out
    def mass(s):
        m=0
        for e in s.elems:
            if e['mat']=='C': m+=EC_RHO*e['sec']['A']*e['L']
        return m

def prop_clearance(X,r_of):
    """min clearance of rod surfaces to prop disk volumes (disk r RD, sweep +-SW)"""
    best=1e9
    for i,p in enumerate(X):
        x,y,z=p
        for mx,my in MOTORS:
            d=np.hypot(x-mx,y-my)-RD; dz=max(0,abs(z)-SW)
            c=(np.hypot(max(d,0),dz) if d>0 else dz)-r_of[i]
            best=min(best,c)
    return best

# ---------------- base.py ----------------
import numpy as np
def params(lower=('solid',3.0),upper=('solid',2.0),strut=('solid',3.0),leg=('solid',3.0),ZL=-8.,ZU=8.,sock_y=50.,legs=True,nose=False,leg_x=77.5,pads='chamfer',pad_x=22.,upper_at_tees=True,sock_L=9.,strut_tail=None,tip_z=-26.5):
    zp=4.0
    struts=[((0,sock_y,22),(0,111,zp),(0,115,zp)),((0,-sock_y,22),(0,-111,zp),(0,-115,zp)),((-55,0,20),(-111,0,zp),(-115,0,zp))]
    tees=[(0,115),(0,-115),(-115,0)]
    if nose:
        struts.append(((55,0,20),(111,0,zp),(115,0,zp))); tees.append((115,0))
    L=[]; extra=[]
    if legs:
        for sx in(1,-1):
            for sy in(1,-1):
                L.append(((sx*leg_x,sy*115,ZL),(sx*leg_x,sy*110,ZL-4),(sx*leg_x,sy*110,tip_z)))
                if leg_x!=77.5: extra.append((sx*leg_x,sy*115))
    P=[(sx*96.25,sy*96.25) for sx in(1,-1) for sy in(1,-1)] if pads=='chamfer' else [(sx*pad_x,sy*115) for sx in(1,-1) for sy in(1,-1)]
    if pads!='chamfer': extra+=P
    return dict(strut_tail=strut_tail,F=115.,C=77.5,ZL=ZL,ZU=ZU,tees=tees,extra=extra,lower=lower,upper=upper,strut=strut,leg=leg,struts=struts,legs=L,pads=P,leg_x=leg_x,upper_at_tees=upper_at_tees,sock_L=sock_L,tip_z=tip_z)

# ---------------- cases.py ----------------
import numpy as np
MIN_KG=0.40; G0=9.81
SOCK_L=9.0; TPU_BEAR=22.4; ROD_ALLOW=622.
def energy(v): return 0.5*MIN_KG*v*v*1000.   # N*mm

def tip_nodes(g):
    """leg rod ends (lowest node of each leg), sorted"""
    tz=g.P.get('tip_z',-26.5)
    return sorted(set(i for e in g.elems if e['tag']=='leg' for i in e['n'] if abs(g.X[i,2]-tz)<1e-6))

def node_radius(g):
    r=np.zeros(g.N)
    for e in g.elems:
        rr=e['sec']['d']/2 if e['mat']=='C' else e['sec']['d']/2
        for i in e['n']: r[i]=max(r[i],rr)
    return r

def ring_nodes(g):
    """plan-ordered ring nodes for lower and upper rings"""
    lo=[];up=[]
    for e in g.elems:
        if e['tag']=='lower': lo+=list(e['n'])
        if e['tag']=='upper': up+=list(e['n'])
    return sorted(set(lo)),sorted(set(up))

def socket_bearing(g,u):
    """max bearing stress at rod ends that sit in TPU nodes: 6M/(d L^2)"""
    tpu_nodes=set()
    for e in g.elems:
        if e['mat']=='T': tpu_nodes.update(e['n'])
    st=g.stresses(u); worst=0; where=None
    for e,(sg,Mends) in zip(g.elems,st):
        if e['mat']!='C': continue
        for k,ni in enumerate(e['n']):
            if ni in tpu_nodes:
                b=6*Mends[k]/(e['sec']['d']*g.P.get('sock_L',SOCK_L)**2)
                if b>worst: worst=b; where=(e['tag'],tuple(np.round(g.X[ni],1)))
    return worst,where

def rod_stress(g,u):
    st=g.stresses(u); worst=0; where=None
    for e,(sg,_) in zip(g.elems,st):
        if e['mat']=='C' and sg>worst: worst=sg; where=(e['tag'],tuple(np.round(g.X[e['n'][0]],1)))
    return worst,where

def hit_cases(g):
    lo,up=ring_nodes(g); X=g.X
    cases=[]
    # walls: sweep direction, contact = outermost ring nodes (both rings)
    seen=set()
    for a in np.arange(0,360,2.5):
        n=np.array([np.cos(np.radians(a)),np.sin(np.radians(a)),0])
        allr=lo+up; h=max(X[i]@n for i in allr)
        ids=tuple(i for i in allr if X[i]@n>=h-0.05)
        key=(ids,round(a,1)) if len(ids)<=2 else (ids,)
        if (ids,) in seen: continue
        seen.add((ids,)); cases.append(('wall',a,list(ids),[n]*len(ids)))
    # poles (vertical) and edges (single ring) at every plan point, along the local outward normal
    plan={}
    for i in lo: plan.setdefault((round(X[i,0],3),round(X[i,1],3)),[None,None])[0]=i
    for i in up: plan.setdefault((round(X[i,0],3),round(X[i,1],3)),[None,None])[1]=i
    for (x,y),(il,iu) in plan.items():
        if il is None or iu is None: continue
        # outward normal from clipped-square geometry
        ax,ay=abs(x),abs(y)
        if ax>=114.99 and ay<=77.51: ns=[np.array([np.sign(x),0,0])]
        elif ay>=114.99 and ax<=77.51: ns=[np.array([0,np.sign(y),0])]
        else: ns=[np.array([np.sign(x),np.sign(y),0])/np.sqrt(2)]
        if abs(ax-115)<1e-3 and abs(ay-77.5)<1e-3 or abs(ax-77.5)<1e-3 and abs(ay-115)<1e-3:
            n1=np.array([np.sign(x),0,0]) if ax>100 else np.array([0,np.sign(y),0])
            n2=np.array([np.sign(x),np.sign(y),0])/np.sqrt(2)
            ns=[n1,(n1+n2)/np.linalg.norm(n1+n2),n2]
        for n in ns:
            cases.append(('pole',(x,y),[il,iu],[n,n]))
            cases.append(('edge-lo',(x,y),[il],[n]))
            cases.append(('edge-up',(x,y),[iu],[n]))
    # leg tips hit horizontally (low obstacle)
    tips=tip_nodes(g)
    for i in set(tips):
        x,y=g.X[i,:2]
        for n in (np.array([0,np.sign(y),0]),np.array([np.sign(x),0,0])):
            cases.append(('leg',(x,y),[i],[n]))
    return cases

def run_hits(g,v=1.0,keep=False):
    R=node_radius(g); base=prop_clearance(g.X,R)
    res=[]
    for kind,where,ids,dirs in hit_cases(g):
        out=g.rigid_indent(ids,[ -0+d for d in dirs],energy(v))  # load pushes outward point inward: use -d
        res.append((kind,where,out))
    return res,base

def evaluate(g,v_hit=1.0,v_touch=0.5,verbose=False):
    R=node_radius(g); base=prop_clearance(g.X,R)
    E1=energy(v_hit)
    worst=dict(clr=1e9,g=0,rod1=0,bear1=0,rodT=0,bearT=0)
    wh={}
    for kind,where,ids,dirs in hit_cases(g):
        # force acts INWARD (obstacle pushes guard toward the drone): direction -n
        o=g.rigid_indent(ids,[-d for d in dirs],E1)
        Xd=g.X+o['u'].reshape(-1,6)[:,:3]
        c=prop_clearance(Xd,R)
        gin=o['F']/(MIN_KG*G0)
        r1,rw=rod_stress(g,o['u']); b1,bw=socket_bearing(g,o['u'])
        s=v_touch/v_hit
        for key,val,w in (('clr',c,None),('g',gin,None),('rod1',r1,rw),('bear1',b1,bw),('rodT',r1*s,rw),('bearT',b1*s,bw)):
            better = val<worst[key] if key=='clr' else val>worst[key]
            if better: worst[key]=val; wh[key]=(kind,where,w)
    return base,worst,wh

def static_cases(g):
    """landing 6 g on two legs and four legs, swap push, detent hold, seat pads at 25 g"""
    out={}
    tips=tip_nodes(g)
    W=MIN_KG*G0
    def apply(loads):
        F=np.zeros(6*g.N)
        for i,f in loads: F[6*i:6*i+3]+=f
        return g.solve(F)
    if tips:
        for name,sel in (('land6g_4legs',tips),('land6g_2legs_side',[t for t in tips if g.X[t,1]>0]),('land6g_2legs_diag',[t for t in tips if g.X[t,0]*g.X[t,1]>0])):
            u=apply([(i,np.array([0,0,6*W/len(sel)])) for i in sel]); out[name]=u
    lo,_=ring_nodes(g)
    pads=[min(lo,key=lambda i:np.hypot(g.X[i,0]-px,g.X[i,1]-py)+abs(g.X[i,2]-g.P['ZL'])) for px,py in g.P['pads']]
    out['swap_push_11N']=apply([(i,np.array([-11/4,0,0])) for i in pads])
    out['seat_25g_on_pads']=apply([(i,np.array([0,0,25*W/4])) for i in pads])
    out['hold_40N_up_reported']=apply([(i,np.array([0,0,-40/4])) for i in pads])
    res={}
    for k,u in out.items():
        r,rw=rod_stress(g,u); b,bw=socket_bearing(g,u)
        U=u.reshape(-1,6)[:,:3]
        tipdz=max([U[i,2] for i in tips] or [0]) if k.startswith('land') else np.abs(U).max()
        res[k]=(r,b,rw,bw,tipdz)
    return res,pads

def node_mass(P):
    tpu=1.22e-3
    SL=P.get('sock_L',9.)
    sock=lambda d:np.pi/4*((d+4.5)**2-(d+0.5)**2)*SL*tpu
    dl=P['lower'][1]; du=P['upper'][1]; ds=P['strut'][1]; dg=P['leg'][1]
    dt=(P.get('strut_tail') or P['strut'])[1]
    corner=2*sock(dl)+2*sock(du)+0.8
    tee=2*sock(dl)+(2*sock(du) if P.get('upper_at_tees',True) else 0)+sock(ds)+1.0
    n_t=len(P['tees']); n_leg=len(P['legs'])
    leg_on_corner=P.get('leg_x',77.5)==77.5
    legnode=(sock(dg)+0.8) if leg_on_corner else (2*sock(dl)+sock(dg)+0.8)
    nglue=8*4+n_t*(5 if P.get('upper_at_tees',True) else 3)+n_leg*(1 if leg_on_corner else 3)
    tee_t=tee-sock(ds)+sock(dt)
    return 8*corner+(n_t-1)*tee+tee_t+n_leg*legnode+0.06*nglue

# ---------------- search.py ----------------
LIM=dict(clr=0.0,rod=622.,bear=22.4,tipdz=5.0)
def full_eval(P):
    g=Guard(P); g.P=P
    base,w,wh=evaluate(g)
    st,_=static_cases(g)
    m=g.mass()+node_mass(P)
    sized=dict(clr1=w['clr'],rodT=w['rodT'],bearT=w['bearT'])
    worst_static_rod=max(v[0] for k,v in st.items() if 'reported' not in k)
    worst_static_bear=max(v[1] for k,v in st.items() if 'reported' not in k)
    tipdz=max(v[4] for k,v in st.items() if k.startswith('land'))
    ok=(w['clr']>=LIM['clr'] and w['rodT']<=LIM['rod'] and w['bearT']<=LIM['bear'] and worst_static_rod<=LIM['rod'] and worst_static_bear<=LIM['bear'] and tipdz<=LIM['tipdz'])
    return dict(mass=m,rods=g.mass(),ok=bool(ok),clr1=w['clr'],g1=w['g'],rod1=w['rod1'],bear1=w['bear1'],rodT=w['rodT'],bearT=w['bearT'],
        static={k:[float(v[0]),float(v[1]),float(v[4])] for k,v in st.items()},static_rod=worst_static_rod,static_bear=worst_static_bear,tipdz=tipdz,
        where={k:str(v) for k,v in wh.items()},base_clr=base)
def slider_sockets_13oct(P):
    """slider chat, 13 Oct: every slider socket above the plate face (the slider prints flat on it). Side struts from
    (0, +-43.9, 30.8) to the same side tees, 72.3 long; the single tail strut from (-55, 0, 20), 2 under the plate face,
    becomes a V of two 3.0 struts from (-62, +-34, 29.5) to the tail tee, 64.9 long."""
    zp=4.0; P=dict(P)
    P['struts']=[((0,43.9,30.8),(0,111,zp),(0,115,zp)),((0,-43.9,30.8),(0,-111,zp),(0,-115,zp)),
                 ((-62,34,29.5),(-111,0,zp),(-115,0,zp)),((-62,-34,29.5),(-111,0,zp),(-115,0,zp))]
    return P

def legacy_main():
    """13 Oct pinwheel-era check (props at z 0, F 115 / C 77.5). Kept for reference; cage v2 is v2_main()."""
    P=params(lower=('solid',2.5),upper=('solid',2.0),strut=('tube',4.0,3.0),strut_tail=('solid',3.0),leg=('solid',2.0),leg_x=30.,upper_at_tees=False)
    r=full_eval(P)
    print('mass',round(r['mass']+0.44,1),'g (incl. 4 tube plugs)  clr1',round(r['clr1'],1),'peak g',round(r['g1'],1),'v_intact',round(min(622/r['rod1'],22.4/r['bear1']),2))
    for k,v in r['static'].items(): print(k,[round(x,1) for x in v])
    P2=slider_sockets_13oct(P); r=full_eval(P2)
    print('slider sockets 13 Oct: mass',round(r['mass']+0.44,1),'clr1',round(r['clr1'],1),'static clr',round(r['base_clr'],1),'peak g',round(r['g1'],1),'v_intact',round(min(622/r['rod1'],22.4/r['bear1']),2))
    for k,v in r['static'].items(): print(k,[round(x,1) for x in v])

# ---------------- cage v2 (17 Oct 2026, cage v2 chat): props on top, prop plane z 41.2 ----------------
"""Cage v2 frame model (F). Geometry from v2/cage_v2/ (cage_cad.py, strut_route.json, foot_opt.json): clipped square
F 112 / C 100; lower ring 2.5 at ZL = prop plane - 8, upper 2.0 at ZU = prop plane + 8, upper continuous over the tees; lower ring
continuous through the tees since 17 Oct (tee_through: the tee is a TPU sleeve; was two rods ending in the tee);
PETG corner nodes (wheel-side path through the core, corner post, brace at the rod ends, mouth posts), TPU tee nodes,
side struts 3.0 from the side hubs (clamped on the side plate bolts), tail strut 3.0 from the tail hub (clamped),
4 TPU floor feet on 3.0 legs to the side hubs.
Skids: skid=None drops them; skid_axis 'x' (17 Oct 03:27, receiver v3 layout: one 2.5 skid per side joining its front
and back foot along x, axis at foot_opt.json skid z) or 'y' (16 Oct: left to right, dropped in pass 2).
17 Oct reinforcement passes on the 1 m/s hits (worst 3.43 -> 1.87): lower ring through the tees, skids off, mouth posts 4.0.
Index = stress / allowable, worst element over every case: carbon 622 (ROD_ALLOW); TPU for AMS 22.4 (Bambu TDS X-Y
tensile; E 1190 X-Y, > 650 % elongation: TPU yields, it does not snap) and its socket bearing 22.4;
PETG crash class (printopt.allowables): in-layer 39.4, across the layers 19.7 (nodes print drone z up, so a member's
across-layer share is its |axis z|). Rod ends in PETG sockets take bearing 6M/(d L^2) against 39.4.
Hits: rigid wall / pole / edge / low obstacle at v (1 m/s design, 402 g build-up -> 0.40 kg), clearance to the swept
disks (r 44.45, prop plane +-4) after the deflection. Statics: floor landing 6 g on 4 / 2 / diagonal feet,
swap push 15 N on the two wheels of one face (catch interface 5)."""
import os
PZ=41.2
EP,GP=2780.,1000.                      # PETG in-layer modulus (printopt), shear E
PETG_XY,PETG_Z=51.0*0.85/1.1,25.5*0.85/1.1
TPU_ALLOW=22.4                         # TPU for AMS X-Y tensile strength (Bambu TDS), same as TPU_BEAR
HERE=os.path.dirname(os.path.abspath(__file__))
def _cage_json(name):
    for d in (os.path.join(HERE,'cage_v2'),HERE):
        p=os.path.join(d,name)
        if os.path.exists(p): return json.load(open(p))
    raise FileNotFoundError(name)

def params_v2(PZ_=PZ,F=112.,C=100.,lower=('solid',2.5),upper=('solid',2.0),strut=('solid',3.0),leg=('solid',3.0),
              skid=('solid',2.5),skid_axis='x',mouth=86.,rod_end=95.,sock_L=12.,route=None,foot=None,tee_through=True,foot_d=5.8,mouth_d=4.0):
    route=route or _cage_json('strut_route.json'); foot=foot or _cage_json('foot_opt.json')
    return dict(PZ=PZ_,F=F,C=C,ZL=PZ_-8.,ZU=PZ_+8.,lower=lower,upper=upper,strut=strut,leg=leg,skid=skid,skid_axis=skid_axis,mouth=mouth,
                rod_end=rod_end,sock_L=sock_L,route=route,foot=foot,tee_flats=(1,2,3),tee_through=tee_through,foot_d=foot_d,mouth_d=mouth_d)

def _rot(k):
    a=k*np.pi/2; return np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1.]])

class Guard2(Guard):
    MAT={'C':(EC,GC),'T':(ET,GT),'P':(EP,GP)}
    def build(s):
        P=s.P; F,C,ZL,ZU=P['F'],P['C'],P['ZL'],P['ZU']; e,m=P['rod_end'],P['mouth']
        s.wheel_pts=[]; s.plan_n={}
        S=lambda d:sec(('solid',d))
        for k in range(4):
            R=_rot(k); tee=k in P['tee_flats']
            ss=[-C,-e]+([-m] if m else [])+([-4.5,0.,4.5] if tee else [0.])+([m] if m else [])+[e,C]
            for z,sc,tag in ((ZL,sec(P['lower']),'lower'),(ZU,sec(P['upper']),'upper')):
                pts=[R@[F,v,z] for v in ss]
                for a,b,va,vb in zip(pts[:-1],pts[1:],ss[:-1],ss[1:]):
                    if abs(va)>=e-1e-9 and abs(vb)>=e-1e-9:                      # node stub between rod end and chamfer
                        s.line(a,b,S(5.2 if tag=='lower' else 4.6),'P','node',n=1)
                    elif tee and tag=='lower' and not P['tee_through'] and abs(va)<=4.5+1e-9 and abs(vb)<=4.5+1e-9:   # tee core, rods end at +-4.5
                        s.line(a,b,S(5.2),'T','tee',n=1)
                    else: s.line(a,b,sc,'C',tag)
            for v in (-e,e):
                s.line(R@[F,v,ZL],R@[F,v,ZU],S(3.6),'P','brace',n=3)                 # brace at the rod ends
            if m:
                for v in (-m,m): s.line(R@[F,v,ZL],R@[F,v,ZU],S(P['mouth_d']),'P','mouth',n=3)  # mouth posts (17 Oct)
            for v in (-C,C): s.wheel_pts.append((R@[F,v,ZL],R@np.array([np.sin(np.pi/4),0,-np.cos(np.pi/4)])))
            for v in ss: s.plan_n[tuple(np.round((R@[F,v,0])[:2],3))]=(R@[1.,0,0])
        # corner: wheel-side path through the core K (106,106), corner post
        for k in range(4):
            R=_rot(k); K=np.array([106.,106.])
            for z,d in ((ZL,4.4),(ZU,4.4)):
                s.line(R@[F,C,z],R@[*K,z],S(d),'P','node',n=2); s.line(R@[*K,z],R@[C,F,z],S(d),'P','node',n=2)
            s.line(R@[*K,ZL],R@[*K,ZU],S(4.8),'P','post',n=3)
        # struts: carbon from the hub (clamped) into the tee socket, TPU from mid-socket to the tee core
        ro=P['route']
        sides=[('side',np.array(ro['side']['bracket_socket_bottom'],float)),
               ('side',np.array(ro['side']['bracket_socket_bottom'],float)*[1,-1,1]),
               ('tail',np.array(ro['tail']['bracket_socket_bottom'],float))]
        s.roots=[]
        for name,B in sides:
            Rp=np.array(ro[name]['ring_point'],float)
            if B[1]<0: Rp=Rp*[1,-1,1]
            Rp[2]=ZL; u=(Rp-B)/np.linalg.norm(Rp-B)
            ids=s.line(B,Rp-8.5*u,sec(P['strut']),'C','strut'); s.fixed.append(ids[0]); s.roots.append(ids[0])
            s.line(Rp-8.5*u,Rp,S(5.8),'T','tee',n=1)
        # feet: legs from the side hubs (same clamped root), TPU foot body, skids
        xf,yf,zc=P['foot']['foot_centre']; s.feet=[]
        zs=P['foot'].get('skid',{}).get('z',zc+1.5)
        for sx in (1,-1):
            for sy in (1,-1):
                B=np.array(ro['side']['bracket_socket_bottom'],float)*[1,sy,1]
                Fc=np.array([sx*xf,sy*yf,zc]); ids=s.line(B,Fc,sec(P['leg']),'C','leg',n=7)
                if P['skid'] and P['skid_axis']=='y':
                    s.line(Fc,[sx*xf,sy*(yf-1.5),zc+1.5],S(P['foot_d']),'T','foot',n=1)
                elif P['skid']:                                                 # skid socket runs inboard along x
                    s.line(Fc,[sx*(xf-2.0),sy*yf,zs],S(P['foot_d']),'T','foot',n=1)
                s.feet.append(ids[-1])
            if P['skid'] and P['skid_axis']=='y':
                s.line([sx*xf,yf-1.5,zc+1.5],[sx*xf,-(yf-1.5),zc+1.5],sec(P['skid']),'C','skid',n=6)
        if P['skid'] and P['skid_axis']=='x':
            for sy in (1,-1):
                s.line([xf-2.0,sy*yf,zs],[-(xf-2.0),sy*yf,zs],sec(P['skid']),'C','skid',n=6)
        s.N=len(s.nodes); s.X=np.array(s.nodes); s.assemble()
    def assemble(s):
        n=6*s.N; K=np.zeros((n,n))
        for e in s.elems:
            E,G=s.MAT[e['mat']]
            k,T,L=frame_k(s.X[e['n'][0]],s.X[e['n'][1]],E,G,e['sec']['A'],e['sec']['I'],e['sec']['J'])
            e['T']=T; e['L']=L; e['E']=E; e['G']=G; e['kl']=T@k@T.T
            dof=np.r_[6*e['n'][0]:6*e['n'][0]+6,6*e['n'][1]:6*e['n'][1]+6]; K[np.ix_(dof,dof)]+=k
        s.K=K; fx=np.zeros(n,bool)
        for i in s.fixed: fx[6*i:6*i+6]=True
        s.free=~fx; s.Kinv=np.linalg.inv(K[np.ix_(s.free,s.free)])
    def node_id(s,p):
        d=np.linalg.norm(s.X-np.asarray(p,float),axis=1); i=int(d.argmin()); assert d[i]<1e-5,(p,d[i]); return i

def prop_clearance2(X,r_of,pz=PZ):
    best=1e9
    for i,(x,y,z) in enumerate(X):
        dz=max(0,abs(z-pz)-SW)
        for mx,my in MOTORS:
            d=np.hypot(x-mx,y-my)-44.45
            best=min(best,(np.hypot(max(d,0),dz) if d>0 else dz)-r_of[i])
    return best

def index_v2(g,u):
    """worst utilisation and where: element stress / allowable, rod-end bearing in TPU / PETG sockets"""
    st=g.stresses(u); host={}
    for e in g.elems:
        if e['mat'] in 'TP':
            for i in e['n']: host[i]='T' if (e['mat']=='T' or host.get(i)=='T') else 'P'
    worst=(0,None)
    for e,(sg,Mends) in zip(g.elems,st):
        if e['mat']=='C': idx=sg/ROD_ALLOW
        elif e['mat']=='T': idx=sg/TPU_ALLOW
        else:
            az=abs((g.X[e['n'][1]]-g.X[e['n'][0]])[2])/e['L']; idx=sg/(PETG_XY*(1-az)+PETG_Z*az)
        w=(e['tag'],tuple(np.round((g.X[e['n'][0]]+g.X[e['n'][1]])/2,1)))
        if idx>worst[0]: worst=(idx,w)
        if e['mat']=='C':
            for k,ni in enumerate(e['n']):
                if ni in host:
                    b=6*Mends[k]/(e['sec']['d']*g.P['sock_L']**2)/(22.4 if host[ni]=='T' else PETG_XY)
                    if b>worst[0]: worst=(b,(e['tag']+' socket',tuple(np.round(g.X[ni],1))))
    return worst

def hit_cases_v2(g):
    lo,up=ring_nodes(g); X=g.X; F,C=g.P['F'],g.P['C']; cases=[]
    seen=set()
    for a in np.arange(0,360,2.5):
        n=np.array([np.cos(np.radians(a)),np.sin(np.radians(a)),0]); allr=lo+up
        h=max(X[i]@n for i in allr); ids=tuple(i for i in allr if X[i]@n>=h-0.05)
        if ids in seen: continue
        seen.add(ids); cases.append(('wall',round(float(a),1),list(ids),[n]*len(ids)))
    plan={}
    for i in lo: plan.setdefault((round(X[i,0],3),round(X[i,1],3)),[None,None])[0]=i
    for i in up: plan.setdefault((round(X[i,0],3),round(X[i,1],3)),[None,None])[1]=i
    for (x,y),(il,iu) in plan.items():
        ax,ay=abs(x),abs(y)
        if ax>F-1e-3 and ay<C-1e-3: ns=[np.array([np.sign(x),0,0])]
        elif ay>F-1e-3 and ax<C-1e-3: ns=[np.array([0,np.sign(y),0])]
        elif (ax>F-1e-3 and abs(ay-C)<1e-3) or (ay>F-1e-3 and abs(ax-C)<1e-3):
            n1=np.array([np.sign(x),0,0]) if ax>F-1e-3 else np.array([0,np.sign(y),0]); n2=np.array([np.sign(x),np.sign(y),0])/np.sqrt(2)
            ns=[n1,(n1+n2)/np.linalg.norm(n1+n2),n2]
        else: ns=[np.array([np.sign(x),np.sign(y),0])/np.sqrt(2)]
        for n in ns:
            if il is not None and iu is not None: cases.append(('pole',(x,y),[il,iu],[n,n]))
            if il is not None: cases.append(('edge-lo',(x,y),[il],[n]))
            if iu is not None: cases.append(('edge-up',(x,y),[iu],[n]))
    for i in g.feet:
        x,y=g.X[i,:2]
        for n in (np.array([0,np.sign(y),0]),np.array([np.sign(x),0,0])): cases.append(('foot',(round(x,1),round(y,1)),[i],[n]))
    return cases

def evaluate_v2(g,v=1.0):
    R=node_radius(g); base=prop_clearance2(g.X,R,g.P['PZ']); E1=energy(v)
    out=dict(clr=(1e9,None),g=(0,None),idx=(0,None),foot=(0,None))
    for kind,where,ids,dirs in hit_cases_v2(g):
        o=g.rigid_indent(ids,[-d for d in dirs],E1)
        c=prop_clearance2(g.X+o['u'].reshape(-1,6)[:,:3],R,g.P['PZ'])
        gin=o['F']/(MIN_KG*G0); idx,w=index_v2(g,o['u'])
        if c<out['clr'][0]: out['clr']=(c,(kind,where))
        if gin>out['g'][0]: out['g']=(gin,(kind,where))
        if idx>out['idx'][0]: out['idx']=(idx,(kind,where,w))
        if kind=='foot' and idx>out['foot'][0]: out['foot']=(idx,(kind,where,w))
    return base,out

def static_v2(g):
    W=MIN_KG*G0; res={}
    def apply(loads):
        Fv=np.zeros(6*g.N)
        for i,f in loads: Fv[6*i:6*i+3]+=f
        return g.solve(Fv)
    ft=g.feet
    for name,sel in (('land6g_4feet',ft),('land6g_2feet_side',[i for i in ft if g.X[i,1]>0]),
                     ('land6g_2feet_front',[i for i in ft if g.X[i,0]>0]),('land6g_2feet_diag',[i for i in ft if g.X[i,0]*g.X[i,1]>0])):
        u=apply([(i,np.array([0,0,6*W/len(sel)])) for i in sel])
        idx,w=index_v2(g,u); dz=max(u.reshape(-1,6)[i,2] for i in sel); res[name]=(idx,w,dz)
    wp=[(p,n) for p,n in g.wheel_pts if n[0]>0.5]                 # the two +x face wheels
    u=apply([(g.node_id(p),-n*7.5) for p,n in wp]); idx,w=index_v2(g,u)
    res['swap_push_15N_2wheels']=(idx,w,float(np.abs(u.reshape(-1,6)[:,:3]).max()))
    return res

def full_eval_v2(P,v=1.0):
    g=Guard2(P); base,h=evaluate_v2(g,v); st=static_v2(g)
    worst=max([(h['idx'][0],'hit '+str(h['idx'][1]))]+[(v_[0],k+' '+str(v_[1])) for k,v_ in st.items()])
    return dict(g=g,base_clr=base,hit=h,static=st,worst=worst,rod_g=g.mass())

def v2_main(pz=PZ,**kw):
    P=params_v2(PZ_=pz,**kw); r=full_eval_v2(P)
    h=r['hit']
    print(f"cage v2 frame model (F), prop plane z {pz}, rings {P['ZL']} / {P['ZU']}, skid {P['skid']} along {P['skid_axis']}, rods in model {r['rod_g']:.1f} g")
    print(f"  static prop clearance {r['base_clr']:.2f} (need 12 incl. bodies; frame model counts rod radius only)")
    print(f"  1 m/s hits: worst index {h['idx'][0]:.2f} at {h['idx'][1]}; peak {h['g'][0]:.1f} g at {h['g'][1]}; min clearance to the disks under load {h['clr'][0]:.2f} at {h['clr'][1]}")
    print(f"  1 m/s foot hits: worst index {h['foot'][0]:.2f} at {h['foot'][1]}")
    for k,(idx,w,d) in r['static'].items(): print(f"  {k}: index {idx:.2f} at {w}; max deflection {d:.2f}")
    print(f"WORST INDEX {r['worst'][0]:.2f}: {r['worst'][1]}")
    return r

if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='legacy': legacy_main()
    elif len(sys.argv)>1 and sys.argv[1]=='noskid': v2_main(skid=None)
    else: v2_main(float(sys.argv[1]) if len(sys.argv)>1 else PZ)
