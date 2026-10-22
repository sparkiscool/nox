# Out-of-plane grillage of 3 mm CF rods inside a rigid pinned frame (rings/edges).
# Compare layouts at worst-case point load: energy to allowable stress, bow at that energy, per gram.
import numpy as np, itertools
E=130e3; G=5e3; d=3.0; I=np.pi*d**4/64; J=np.pi*d**4/32; c=d/2; rho=1.55e-3; A=np.pi*d*d/4
SA=1000.0  # MPa allowable
def seg_inter(p1,p2,q1,q2):
    p1,p2,q1,q2=map(np.array,(p1,p2,q1,q2)); r=p2-p1; s=q2-q1; den=r[0]*s[1]-r[1]*s[0]
    if abs(den)<1e-9: return None
    t=((q1-p1)[0]*s[1]-(q1-p1)[1]*s[0])/den; u=((q1-p1)[0]*r[1]-(q1-p1)[1]*r[0])/den
    if 1e-6<t<1-1e-6 and 1e-6<u<1-1e-6: return t,u
    return None
def analyse(rods, joint='lash', ne=8, edge_fix=False):
    # rods: list of (p1,p2) with ends on frame (pinned w=0)
    # build stations along each rod: ends, crossings, subdivisions
    st=[set([0.0,1.0]) for _ in rods]; cross=[]
    for i,j in itertools.combinations(range(len(rods)),2):
        r=seg_inter(*rods[i],*rods[j])
        if r: st[i].add(r[0]); st[j].add(r[1]); cross.append((i,r[0],j,r[1]))
    nodes=[]; nid={}; elems=[]
    for i,(p1,p2) in enumerate(rods):
        ts=sorted(st[i]); full=[]
        for a,b in zip(ts[:-1],ts[1:]):
            full+= list(np.linspace(a,b,ne+1)[:-1])
        full.append(1.0)
        ids=[]
        for t in full:
            nid[(i,round(t,9))]=len(nodes); nodes.append((i,t,np.array(p1)+t*(np.array(p2)-np.array(p1)))); ids.append(len(nodes)-1)
        for a,b in zip(ids[:-1],ids[1:]): elems.append((a,b))
    N=len(nodes); ndof=3*N  # w, thx, thy
    K=np.zeros((ndof,ndof))
    for a,b in elems:
        pa,pb=nodes[a][2],nodes[b][2]; L=np.linalg.norm(pb-pa); cx,cy=(pb-pa)/L
        kb=E*I/L**3*np.array([[12,6*L,-12,6*L],[6*L,4*L*L,-6*L,2*L*L],[-12,-6*L,12,-6*L],[6*L,2*L*L,-6*L,4*L*L]])
        kt=G*J/L*np.array([[1,-1],[-1,1]])
        # local: w, bending rot (about in-plane perpendicular), torsion rot (about axis)
        # bending rotation = dw/ds = thx*? use theta vector (thx,thy) = rotation components; slope along s: dw/ds = -thx*cy + thy*cx (convention)
        T=np.zeros((6,6))
        for k,n0 in enumerate([0,3]):
            T[3*k,n0]=1
            T[3*k+1,n0+1]=-cy; T[3*k+1,n0+2]=cx  # bending rot
            T[3*k+2,n0+1]=cx; T[3*k+2,n0+2]=cy   # torsion
        kl=np.zeros((6,6)); bi=[0,1,3,4]; ti=[2,5]
        kl[np.ix_(bi,bi)]=kb; kl[np.ix_(ti,ti)]=kt
        kg=T.T@kl@T; idx=[3*a,3*a+1,3*a+2,3*b,3*b+1,3*b+2]; K[np.ix_(idx,idx)]+=kg
    pen=1e8
    for (i,ti,j,tj) in cross:
        a=nid[(i,round(ti,9))]; b=nid[(j,round(tj,9))]
        dofs=[0] if joint=='lash' else [0,1,2]
        for k in dofs:
            ia,ib=3*a+k,3*b+k; K[ia,ia]+=pen; K[ib,ib]+=pen; K[ia,ib]-=pen; K[ib,ia]-=pen
    K[np.arange(ndof)%3!=0, np.arange(ndof)%3!=0] += 1e-2
    fixed=[]
    for n,(i,t,p) in enumerate(nodes):
        if t in (0.0,1.0):
            fixed.append(3*n)
            if edge_fix: fixed += [3*n+1,3*n+2]
    free=np.array([k for k in range(ndof) if k not in set(fixed)])
    Kf=K[np.ix_(free,free)]; Kinv=np.linalg.inv(Kf)
    pos={k:i for i,k in enumerate(free)}
    # stress per unit load: from element end moments
    worst=None
    loadnodes=[n for n in range(N) if 3*n in pos]
    res=[]
    for n in loadnodes:
        F=np.zeros(len(free)); F[pos[3*n]]=1
        u=np.zeros(ndof); u[free]=Kinv@F
        f=u[3*n]
        smax=0
        for a,b in elems:
            pa,pb=nodes[a][2],nodes[b][2]; L=np.linalg.norm(pb-pa); cx,cy=(pb-pa)/L
            ua=u[[3*a,3*a+1,3*a+2]]; ub=u[[3*b,3*b+1,3*b+2]]
            ra=-cy*ua[1]+cx*ua[2]; rb=-cy*ub[1]+cx*ub[2]
            ta=cx*ua[1]+cy*ua[2]; tb=cx*ub[1]+cy*ub[2]
            kb=E*I/L**3*np.array([[12,6*L,-12,6*L],[6*L,4*L*L,-6*L,2*L*L],[-12,-6*L,12,-6*L],[6*L,2*L*L,-6*L,4*L*L]])
            fe=kb@np.array([ua[0],ra,ub[0],rb]); M=max(abs(fe[1]),abs(fe[3]))
            Tq=abs(G*J/L*(tb-ta)); s=np.hypot(M*c/I, 0)  # bending normal stress; torsion shear reported separately
            smax=max(smax,s)
        U=SA**2*f/(2*smax**2); delta=SA*f/smax; Pmax=SA/smax
        res.append((U,delta,Pmax,nodes[n][2]))
    Umin=min(res,key=lambda r:r[0])
    length=sum(np.linalg.norm(np.array(b)-np.array(a)) for a,b in rods)
    return Umin, length, len(cross), res
a,b=200.0,100.0  # panel 200 x 100 mm, frame pinned
L={}
L['A open quads: 3 rods across, s=50']=[((x,0),(x,b)) for x in (50,100,150)]
L['B open quads: 1 long rod mid']=[((0,50),(200,50))]
L['C grid 3 across + 1 long (lashed)']=[((x,0),(x,b)) for x in (50,100,150)]+[((0,50),(200,50))]
L['D X diagonals in 2 cells (lashed)']=[((0,0),(100,100)),((0,100),(100,0)),((100,0),(200,100)),((100,100),(200,0))]
L['E Warren triangles: 4 diagonals (zigzag)']=[((0,0),(50,100)),((50,100),(100,0)),((100,0),(150,100)),((150,100),(200,0))]
L['F two-way diagonal lattice (lashed)']=[((0,0),(100,100)),((50,0),(150,100)),((100,0),(200,100)),((0,100),(100,0)),((50,100),(150,0)),((100,100),(200,0))]
for name,rods in L.items():
    for joint in (['lash','rigid'] if 'lashed' in name else ['lash']):
        (U,dl,P,p),length,nc,res=analyse(rods,joint)
        m=length*A*rho
        print(f"{name:42s} {joint:5s} rod {length:6.0f} mm {m:4.1f} g  cross {nc}  worstU {U/1000:5.3f} J at {p.round(0)}  bow {dl:4.1f} mm  P {P:4.0f} N  J/g {U/1000/m:5.3f}")
