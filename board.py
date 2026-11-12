import math
from sexp import *
CU=['F.Cu','In1.Cu','In2.Cu','B.Cu']
def num(x): return float(x)
class Pad: pass
def load(path):
    root=parse(open(path).read())[0]
    pads=[]
    for fp in find(root,'footprint'):
        at=find1(fp,'at'); X,Y=num(at[1]),num(at[2]); th=num(at[3]) if len(at)>3 else 0.0
        ref=[p for p in find(fp,'property') if p[1]=='Reference'][0][2]
        fl=find1(fp,'layer')[1]
        t=math.radians(th); c,s=math.cos(t),math.sin(t)
        for p in find(fp,'pad'):
            pa=find1(p,'at'); x,y=num(pa[1]),num(pa[2]); pang=num(pa[3]) if len(pa)>3 else th
            P=Pad(); P.ref=ref; P.num=p[1]; P.kind=str(p[2]); P.shape=str(p[3])
            P.x=X+x*c+y*s; P.y=Y-x*s+y*c; P.ang=pang
            sz=find1(p,'size'); P.w,P.h=num(sz[1]),num(sz[2])
            dr=find1(p,'drill'); P.drill=0.0; P.drillw=P.drillh=0.0
            if dr:
                vals=[v for v in dr[1:] if not isinstance(v,list) and str(v)!='oval']
                if vals:
                    P.drillw=num(vals[0]); P.drillh=num(vals[1]) if len(vals)>1 else P.drillw
                    P.drill=min(P.drillw,P.drillh)
            lays=[str(l) for l in find1(p,'layers')[1:]]
            if '*.Cu' in lays: P.layers=list(CU)
            else: P.layers=[l for l in lays if l in CU]
            if P.kind=='np_thru_hole': P.layers=[]
            n=find1(p,'net'); P.net=n[1] if n else ''
            P.fplayer=fl
            pads.append(P)
    return root,pads
