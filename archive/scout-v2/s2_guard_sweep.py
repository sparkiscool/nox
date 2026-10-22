"""S2 guard variation sweep model (guard chat, 13 Oct 2026). Separate from v2/s2_guard_frame.py (the shared check
script that v2/box_seat_checks.py imports); same beam model and hit cases, plus: legs anywhere / any length, groove and
chamfer catch cases, socket-mouth vs span allowables (sleeved spans 1000 MPa), rocking compliance at the slider sockets
(rock_k, a TPU 95A liner), one-ring option. Results: nox/v2/s2-guard.md "Variations tested (13 Oct)".
Run: python3 s2_guard_sweep.py  -> the settled design (CH) and the 12 Oct layout (DOC), socket reactions, one-ring ref."""
"""Model notes.
New: 13 Oct slider sockets as default; legs anywhere / any length; groove-seat and chamfer-seat catch cases;
socket-mouth vs span allowables (sleeved spans 1000 MPa, research rule 6); optional rocking compliance at the
slider sockets (TPU 95A sleeve); coverage metric. Units N, mm, MPa; masses in g. Tags: C model, E estimate."""
import numpy as np, json, itertools, sys, time
M_=54.1; RD=44.5; SW=4.0
MOTORS=[(sx*M_,sy*M_) for sx in(1,-1) for sy in(1,-1)]
EC,GC,RHO_C=125000.,5000.,1.55e-3      # carbon (E)
ET,GT,RHO_T=1190.,400.,1.22e-3         # TPU for AMS (datasheet XY, CHECK)
MIN_KG=0.40; G0=9.81; W=MIN_KG*G0
A_MOUTH=622.; A_SPAN_BARE=622.; A_SPAN_SLEEVED=1000.; TPU_BEAR=22.4

def sec(s):
    if s[0]=='solid': d=s[1]; A=np.pi*d*d/4; I=np.pi*d**4/64
    else: do,di=s[1],s[2]; A=np.pi*(do*do-di*di)/4; I=np.pi*(do**4-di**4)/64
    return dict(A=A,I=I,J=2*I,c=s[1]/2,d=s[1])

def frame_k(p1,p2,E,G,A,I,J):
    L=np.linalg.norm(p2-p1); ex=(p2-p1)/L
    ref=np.array([0,0,1.]) if abs(ex[2])<0.95 else np.array([1.,0,0])
    ey=np.cross(ref,ex); ey/=np.linalg.norm(ey); ez=np.cross(ex,ey)
    R=np.vstack([ex,ey,ez]); T=np.zeros((12,12))
    for i in range(4): T[3*i:3*i+3,3*i:3*i+3]=R
    k=np.zeros((12,12)); a=E*A/L; t=G*J/L
    b1=12*E*I/L**3; b2=6*E*I/L**2; b3=4*E*I/L; b4=2*E*I/L
    k[0,0]=k[6,6]=a; k[0,6]=k[6,0]=-a; k[3,3]=k[9,9]=t; k[3,9]=k[9,3]=-t
    for (v,r,s_) in ((1,5,1),(2,4,-1)):
        k[v,v]=k[v+6,v+6]=b1; k[v,v+6]=k[v+6,v]=-b1
        k[v,r]=k[r,v]=s_*b2; k[v,r+6]=k[r+6,v]=s_*b2
        k[v+6,r]=k[r,v+6]=-s_*b2; k[v+6,r+6]=k[r+6,v+6]=-s_*b2
        k[r,r]=k[r+6,r+6]=b3; k[r,r+6]=k[r+6,r]=b4
    return k,T,L

def P_(lower=('solid',2.5),upper=('solid',2.0),side=('tube',4.0,3.0),tail=('solid',3.0),leg=('solid',2.0),
       ZL=-8.,ZU=8.,leg_x=30.,tip_z=-29.5,legs=True,upper_at_tees=False,sock_L=9.,rock_k=None,sleeved=False,
       seat='chamfer',F=115.,C=77.5,single=False,vleg_bx=None,vleg=('solid',2.0)):
    zp=4.0
    struts=[((0,43.9,30.8),(0,111,zp),(0,115,zp),side),((0,-43.9,30.8),(0,-111,zp),(0,-115,zp),side),
            ((-62,34,29.5),(-111,0,zp),(-115,0,zp),tail),((-62,-34,29.5),(-111,0,zp),(-115,0,zp),tail)]
    tees=[(0,F),(0,-F),(-F,0)]
    return dict(lower=lower,upper=upper,side=side,tail=tail,leg=leg,ZL=ZL,ZU=ZU,leg_x=leg_x,tip_z=tip_z,legs=legs,
                upper_at_tees=upper_at_tees,sock_L=sock_L,rock_k=rock_k,sleeved=sleeved,seat=seat,F=F,C=C,
                struts=struts,tees=tees,single=single,vleg_bx=vleg_bx,vleg=vleg)

class Guard:
    def __init__(s,P):
        s.P=P; s.nodes=[]; s.elems=[]; s.fixed=[]; s.tips=[]; s.tpu=set(); s.roots=[]
        s.build(); s.assemble()
    def node(s,p):
        p=np.asarray(p,float)
        for i,q in enumerate(s.nodes):
            if np.linalg.norm(q-p)<1e-6: return i
        s.nodes.append(p); return len(s.nodes)-1
    def line(s,a,b,secd,mat,tag,n=None,ln=10.,E=None):
        a=np.asarray(a,float); b=np.asarray(b,float); L=np.linalg.norm(b-a)
        n=n or max(2,int(np.ceil(L/ln)))
        ids=[s.node(a+(b-a)*i/n) for i in range(n+1)]
        for i in range(n): s.elems.append(dict(n=(ids[i],ids[i+1]),sec=secd,mat=mat,tag=tag,Eo=E))
        if mat!='C': s.tpu.update(ids)
        return ids
    def build(s):
        P=s.P; F,C=P['F'],P['C']; ZL,ZU=P['ZL'],P['ZU']
        oct8=[(F,-C),(F,C),(C,F),(-C,F),(-F,C),(-F,-C),(-C,-F),(C,-F)]
        extra=[]
        lx=P['leg_x']
        if P['legs'] and abs(lx-C)>1e-6: extra=[(sx*lx,sy*F) for sx in(1,-1) for sy in(1,-1)]
        if P['legs'] and P.get('vleg_bx'): extra=extra+[(sx*P['vleg_bx'],sy*F) for sx in(1,-1) for sy in(1,-1)]
        pts=[]
        for i in range(8):
            a=np.array(oct8[i]);b=np.array(oct8[(i+1)%8]);pts.append(('c',tuple(a)))
            seg=[]
            for kind,lst in (('t',P['tees']),('x',extra)):
                for t in lst:
                    t=np.array(t,float);ab=b-a;u=np.dot(t-a,ab)/np.dot(ab,ab)
                    if 0<u<1 and np.linalg.norm(a+u*ab-t)<1e-6: seg.append((u,kind,tuple(t)))
            for u,kind,t in sorted(seg): pts.append((kind,t))
        s.ringpts=pts
        rings=[(ZL,sec(P['lower']),'lower')]
        if not P['single']: rings.append((ZU,sec(P['upper']),'upper'))
        for z,sc,tag in rings:
            for i in range(len(pts)):
                s.line((*pts[i][1],z),(*pts[(i+1)%len(pts)][1],z),sc,'C',tag)
        for kind,(x,y) in pts:
            if P['single']:
                if kind in('t','x'): s.line((x,y,ZL),(x,y,ZL+0.01*0+4.0) if kind=='t' else (x,y,ZL+3.5),sec(('solid',8.8 if kind=='t' else 7.0)),'T','post',n=2)
                else: s.line((x,y,ZL-3.75),(x,y,ZL+3.75),sec(('solid',7.5)),'T','post',n=2)
                continue
            if kind=='x': s.line((x,y,ZL),(x,y,ZL-3.5),sec(('solid',7.0)),'T','post',n=1); continue
            d=7.5 if kind=='c' else 8.8
            if kind=='t' and not P['upper_at_tees']: s.line((x,y,ZL),(x,y,4.0),sec(('solid',d)),'T','post',n=3)
            else: s.line((x,y,ZL),(x,y,ZU),sec(('solid',d)),'T','post',n=4)
        done=set()
        for (sock,att,post,st) in P['struts']:
            if P['rock_k']:
                base=s.node(np.array(sock)+0.0); s.fixed.append(base)
                r=s.node(np.array(sock)+np.array([0,0,1e-3]))   # coincident root joined by a spring
                s.elems.append(dict(n=(base,r),mat='S',tag='rock',k=P['rock_k'],sec=sec(st)))
                ids=s.line(np.array(sock)+np.array([0,0,1e-3]),att,sec(st),'C','strut'); s.roots.append(base)
            else:
                ids=s.line(sock,att,sec(st),'C','strut'); s.fixed.append(ids[0]); s.roots.append(ids[0])
            if (att,post) not in done: s.line(att,post,sec(('solid',8.0)),'T','link',n=1); done.add((att,post))
        if P['legs']:
            Lg=sec(P['leg'])
            for sx in(1,-1):
                for sy in(1,-1):
                    x=sx*P['leg_x']; top=(x,sy*(F-5),ZL-4)
                    s.line((x,sy*F,ZL),top,sec(('solid',7.0)),'T','link',n=1)
                    ids=s.line(top,(x,sy*(F-5),P['tip_z']+3),Lg,'C','leg',n=4)
                    s.tips.append(ids[-1])
                    if P.get('vleg_bx'):
                        s.line((sx*P['vleg_bx'],sy*F,ZL),(x,sy*(F-5),P['tip_z']+3),sec(P['vleg']),'C','vleg',n=4)
        s.N=len(s.nodes); s.X=np.array(s.nodes)
    def assemble(s):
        n=6*s.N; K=np.zeros((n,n))
        for e in s.elems:
            i,j=e['n']; dof=np.r_[6*i:6*i+6,6*j:6*j+6]
            if e['mat']=='S':
                kr=e['k']; kd=np.diag([1e6,1e6,1e6,kr,kr,1e7])   # stiff translation and twist, soft rocking about x/y (E)
                k=np.block([[kd,-kd],[-kd,kd]]); K[np.ix_(dof,dof)]+=k; continue
            E,G=(EC,GC) if e['mat']=='C' else (ET,GT)
            k,T,L=frame_k(s.X[i],s.X[j],E,G,e['sec']['A'],e['sec']['I'],e['sec']['J'])
            e['T']=T; e['L']=L; e['kl']=k
            K[np.ix_(dof,dof)]+=T.T@k@T
        s.K=K; fx=np.zeros(n,bool)
        for i in s.fixed: fx[6*i:6*i+6]=True
        s.free=~fx; s.Kinv=np.linalg.inv(K[np.ix_(s.free,s.free)])
    def solve(s,F):
        u=np.zeros(6*s.N); u[s.free]=s.Kinv@F[s.free]; return u
    def flex(s,ids,dirs):
        cols=[]
        for i,d in zip(ids,dirs):
            F=np.zeros(6*s.N); F[6*i:6*i+3]=d; cols.append(s.solve(F))
        Cm=np.array([[np.dot(cols[j][6*i:6*i+3],d) for j in range(len(ids))] for i,d in zip(ids,dirs)])
        return Cm,cols
    def indent(s,ids,dirs,energy):
        act=list(range(len(ids))); Cm,cols=s.flex(ids,dirs)
        while True:
            f=np.linalg.solve(Cm[np.ix_(act,act)],np.ones(len(act)))
            if (f>=-1e-9).all(): break
            act=[a for a,v in zip(act,f) if v>1e-9]
        k=f.sum(); dl=np.sqrt(2*energy/k)
        u=sum(f[q]*dl*cols[a] for q,a in enumerate(act))
        return dict(k=k,delta=dl,F=k*dl,u=u)
    def check(s,u):
        """worst utilisations: rod (mouth/span), socket bearing; returns (util, rod_mouth, rod_span, bear, where)"""
        mouths=s.tpu|set(s.fixed)|set(s.roots)
        span_allow=A_SPAN_SLEEVED if s.P['sleeved'] else A_SPAN_BARE
        worst=dict(mouth=0,span=0,bear=0); wh={}
        for e in s.elems:
            if e['mat']!='C': continue
            i,j=e['n']; dof=np.r_[6*i:6*i+6,6*j:6*j+6]
            fl=e['kl']@(e['T']@u[dof])
            for end,ni in ((0,i),(6,j)):
                M=np.hypot(fl[end+4],fl[end+5]); sg=abs(fl[end])/e['sec']['A']+M*e['sec']['c']/e['sec']['I']
                key='mouth' if ni in mouths else 'span'
                if sg>worst[key]: worst[key]=sg; wh[key]=(e['tag'],tuple(np.round(s.X[ni],1)))
                if ni in s.tpu:
                    b=6*M/(e['sec']['d']*s.P['sock_L']**2)
                    if b>worst['bear']: worst['bear']=b; wh['bear']=(e['tag'],tuple(np.round(s.X[ni],1)))
        util=max(worst['mouth']/A_MOUTH,worst['span']/span_allow,worst['bear']/TPU_BEAR)
        return util,worst,wh
    def mass(s):
        return sum(RHO_C*e['sec']['A']*e['L'] for e in s.elems if e['mat']=='C')

def radii(g):
    r=np.zeros(g.N)
    for e in g.elems:
        if e['mat']=='S': continue
        for i in e['n']: r[i]=max(r[i],e['sec']['d']/2)
    return r

def prop_clear(X,r):
    best=1e9
    for p,ri in zip(X,r):
        x,y,z=p
        for mx,my in MOTORS:
            d=np.hypot(x-mx,y-my)-RD; dz=max(0,abs(z)-SW)
            best=min(best,(np.hypot(max(d,0),dz) if d>0 else dz)-ri)
    return best

def ring_ids(g):
    lo=set();up=set()
    for e in g.elems:
        if e['tag']=='lower': lo.update(e['n'])
        if e['tag']=='upper': up.update(e['n'])
    return sorted(lo),sorted(up)

def hit_cases(g):
    lo,up=ring_ids(g); X=g.X; cases=[]; seen=set(); allr=lo+up
    for a in np.arange(0,360,2.5):
        n=np.array([np.cos(np.radians(a)),np.sin(np.radians(a)),0])
        h=max(X[i]@n for i in allr); ids=tuple(i for i in allr if X[i]@n>=h-0.05)
        if ids in seen: continue
        seen.add(ids); cases.append(('wall',a,list(ids),[n]*len(ids)))
    plan={}
    for i in lo: plan.setdefault((round(X[i,0],3),round(X[i,1],3)),[None,None])[0]=i
    for i in up: plan.setdefault((round(X[i,0],3),round(X[i,1],3)),[None,None])[1]=i
    F,C=g.P['F'],g.P['C']
    for (x,y),(il,iu) in plan.items():
        ax,ay=abs(x),abs(y)
        if ax>=F-0.01 and ay<=C+0.01: ns=[np.array([np.sign(x),0,0])]
        elif ay>=F-0.01 and ax<=C+0.01: ns=[np.array([0,np.sign(y),0])]
        else: ns=[np.array([np.sign(x),np.sign(y),0])/np.sqrt(2)]
        if (abs(ax-F)<1e-3 and abs(ay-C)<1e-3) or (abs(ax-C)<1e-3 and abs(ay-F)<1e-3):
            n1=np.array([np.sign(x),0,0]) if ax>F-1 else np.array([0,np.sign(y),0]); n2=np.array([np.sign(x),np.sign(y),0])/np.sqrt(2)
            ns=[n1,(n1+n2)/np.linalg.norm(n1+n2),n2]
        for n in ns:
            if il is not None and iu is not None: cases.append(('pole',(x,y),[il,iu],[n,n]))
            if il is not None: cases.append(('edge-lo',(x,y),[il],[n]))
            if iu is not None: cases.append(('edge-up',(x,y),[iu],[n]))
    for i in g.tips:
        x,y=g.X[i,:2]
        for n in (np.array([0,np.sign(y),0]),np.array([np.sign(x),0,0])): cases.append(('leg',(x,y),[i],[n]))
    return cases

def evaluate(P,v=1.0,detail=False):
    g=Guard(P); R=radii(g); base=prop_clear(g.X,R)
    E1=0.5*MIN_KG*v*v*1000.
    clr=1e9; gmax=0; util1=0; where={}; worst_case=None
    for kind,wh,ids,dirs in hit_cases(g):
        o=g.indent(ids,[-d for d in dirs],E1)
        c=prop_clear(g.X+o['u'].reshape(-1,6)[:,:3],R)
        u_,w_,whs=g.check(o['u'])
        gg=o['F']/W
        if c<clr: clr=c; where['clr']=(kind,np.round(wh,1).tolist() if not np.isscalar(wh) else wh)
        if gg>gmax: gmax=gg; where['g']=(kind,np.round(wh,1).tolist() if not np.isscalar(wh) else wh)
        if u_>util1: util1=u_; where['util']=(kind,np.round(wh,1).tolist() if not np.isscalar(wh) else wh,whs)
    v_int=1.0/np.sqrt(util1)*v if util1>0 else 9   # energy method: stress ~ v
    v_int=v/util1
    # statics
    st={}
    def apply(loads):
        Fv=np.zeros(6*g.N)
        for i,f in loads: Fv[6*i:6*i+3]+=f
        return g.solve(Fv)
    tips=g.tips
    lo,_=ring_ids(g)
    hooks=[min(lo,key=lambda i:np.hypot(g.X[i,0]-sx*96.25,g.X[i,1]-sy*96.25)) for sx in(1,-1) for sy in(1,-1)]
    if tips:
        for name,sel in (('land6g_4',tips),('land6g_2side',[t for t in tips if g.X[t,1]>0]),
                         ('land6g_2front',[t for t in tips if g.X[t,0]>0]),('land6g_2diag',[t for t in tips if g.X[t,0]*g.X[t,1]>0])):
            u=apply([(i,np.array([0,0,6*W/len(sel)])) for i in sel]); ut,_,_=g.check(u)
            st[name]=(ut,max(u[6*i+2] for i in sel))
    if P['seat']=='chamfer':
        o=g.indent(hooks,[np.array([0,0,1.])]*4,W*62.5); ut,_,_=g.check(o['u']); st['catch_drop']=(ut,o['F']/W)
        u=apply([(i,np.array([-15/4,0,0])) for i in hooks]); st['push15']=(g.check(u)[0],0)
    else:
        h=10+34.75
        o=g.indent(tips,[np.array([0,0,1.])]*4,W*h); ut,_,_=g.check(o['u']); st['catch_drop']=(ut,o['F']/W)
        fl=np.array([0,0,1.]); 
        for name,sel,sgn in (('flank6g_2',[t for t in tips if g.X[t,1]>0],1),):
            lat=np.sin(np.radians(27.8)); up=np.cos(np.radians(27.8))
            u=apply([(i,6*W/2/up*np.array([0,-lat*np.sign(g.X[i,1]),up])) for i in sel]); st[name]=(g.check(u)[0],0)
        u=apply([(i,np.array([15/4,0,0])) for i in tips]+[(i,np.array([0,0,-16/4])) for i in hooks]); st['push15_hooks16']=(g.check(u)[0],0)
    m=g.mass()+node_mass(P)
    res=dict(mass=m,rods=g.mass(),base=base,clr1=clr,g1=gmax,util1=util1,v_int=v_int,static=st,where=where)
    if detail: res['g']=g
    return res

def node_mass(P):
    SL=P['sock_L']
    sock=lambda d:np.pi/4*((d+4.5)**2-(d+0.5)**2)*SL*RHO_T*1000/1000
    dl=P['lower'][1]; du=P['upper'][1]; ds=P['side'][1]; dt=P['tail'][1]; dg=P['leg'][1]
    if P['single']:
        corner=2*sock(dl)+0.6; tee_s=2*sock(dl)+sock(ds)+0.9; tee_t=2*sock(dl)+2*sock(dt)+1.0
    else:
        corner=2*sock(dl)+2*sock(du)+0.8
        tee_s=2*sock(dl)+(2*sock(du) if P['upper_at_tees'] else 0)+sock(ds)+1.0
        tee_t=tee_s-sock(ds)+2*sock(dt)
    m=8*corner+2*tee_s+tee_t
    nglue=8*(2 if P['single'] else 4)+2*(3 if not P['upper_at_tees'] else 5)+4
    if P['legs']:
        if abs(P['leg_x']-P['C'])<1e-6: m+=4*(sock(dg)+0.3); nglue+=4
        else: m+=4*(2*sock(dl)+sock(dg)+0.8); nglue+=12
        m+=4*0.25   # hard tips (PETG dome, E)
        if P.get('vleg_bx'): m+=4*(2*sock(dl)+sock(P['vleg'][1])+0.8)+4*sock(P['vleg'][1]); nglue+=4*4
    m+=0.06*nglue
    if P['side'][0]=='tube': m+=0.44
    if P['sleeved']:
        L=832*(1 if P['single'] else 2)/1000; m+=1.3*L
    if P['rock_k']: m+=4*0.3
    return m

# ---------------- settled design check ----------------
CH=dict(lower=('solid',2.5),upper=('solid',2.0),side=('tube',4.0,3.0),tail=('solid',3.0),leg=('solid',2.5),leg_x=77.5,tip_z=-49.5,seat='groove')
DOC=dict(lower=('solid',2.5),upper=('solid',2.0),side=('tube',4.0,3.0),tail=('solid',3.0),leg=('solid',2.0),leg_x=30.,tip_z=-29.5,seat='chamfer')
def reactions(P,v=0.67):
    g=Guard(P); W_=W
    def reac(u):
        R=g.K@u; return [(np.linalg.norm(R[6*i:6*i+3]),np.linalg.norm(R[6*i+3:6*i+6])) for i in g.roots]
    out={}
    worst=[[0,0] for _ in g.roots]
    for kind,wh,ids,dirs in hit_cases(g):
        o=g.indent(ids,[-d for d in dirs],0.5*MIN_KG*v*v*1000)
        for j,(f,m) in enumerate(reac(o['u'])): worst[j][0]=max(worst[j][0],f); worst[j][1]=max(worst[j][1],m)
    out['hits_%.2f'%v]=worst
    def apply(loads):
        Fv=np.zeros(6*g.N)
        for i,f in loads: Fv[6*i:6*i+3]+=f
        return g.solve(Fv)
    tips=g.tips; w=[[0,0] for _ in g.roots]
    for sel in (tips,[t for t in tips if g.X[t,1]>0],[t for t in tips if g.X[t,0]>0],[t for t in tips if g.X[t,0]*g.X[t,1]>0]):
        u=apply([(i,np.array([0,0,6*W_/len(sel)])) for i in sel])
        for j,(f,m) in enumerate(reac(u)): w[j][0]=max(w[j][0],f); w[j][1]=max(w[j][1],m)
    out['landing6g']=w
    if P['seat']=='groove':
        o=g.indent(tips,[np.array([0,0,1.])]*4,W_*44.75); out['catch']=reac(o['u'])
    else:
        lo,_=ring_ids(g); hooks=[min(lo,key=lambda i:np.hypot(g.X[i,0]-sx*96.25,g.X[i,1]-sy*96.25)) for sx in(1,-1) for sy in(1,-1)]
        o=g.indent(hooks,[np.array([0,0,1.])]*4,W_*62.5); out['catch']=reac(o['u'])
    return out,[tuple(np.round(g.X[i],1)) for i in g.roots]
if __name__=='__main__':
    for name,P in (('chosen',CH),('doc',DOC)):
        r=evaluate(P_(**P))
        print(name,'mass',round(float(r['mass']),1),'rods',round(float(r['rods']),1),'base',round(float(r['base']),1),'clr1',round(float(r['clr1']),1),'g1',round(float(r['g1']),1),'v_int',round(float(r['v_int']),2))
        for k,v in r['static'].items(): print('   ',k,[round(float(x),2) for x in v])
        print('   where',r['where'])
        rc,roots=reactions(P_(**P))
        print('   roots',roots)
        for k,v in rc.items(): print('   ',k,[(round(a,1),round(b)) for a,b in v])
    P=P_(**dict(CH,single=True,ZL=0.,lower=('solid',3.0)))
    r=evaluate(P); print('single ring 3.0 at z0: mass',round(float(r['mass']),1),'clr1',round(float(r['clr1']),1),'v',round(float(r['v_int']),2),'g',round(float(r['g1']),1))
