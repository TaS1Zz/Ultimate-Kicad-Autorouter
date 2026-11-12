import pickle, sys, math
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from board import load, CU
tracks,vias,fail=pickle.load(open(sys.argv[1],'rb'))[:3]
_,pads=load(sys.argv[2])
x0,x1,y0,y1=[float(v) for v in sys.argv[3:7]]
fig,ax=plt.subplots(figsize=(16,16*(y1-y0)/(x1-x0)+1))
col={0:'red',2:'orange',3:'blue'}
for p in pads:
    if not (x0-3<p.x<x1+3 and y0-3<p.y<y1+3): continue
    w,h=(p.w,p.h) if round(p.ang)%180==0 else (p.h,p.w)
    c='green' if 'F.Cu' in p.layers and 'B.Cu' in p.layers else ('salmon' if p.layers==['F.Cu'] else ('lightblue' if p.layers else 'gray'))
    ax.add_patch(Rectangle((p.x-w/2,p.y-h/2),w,h,color=c,alpha=0.6))
    if p.net and w*h<3: ax.text(p.x,p.y,p.net[:6],fontsize=4,ha='center',va='center')
for (n,l,a,b,c,d,w) in tracks:
    ax.plot([a,c],[b,d],color=col[l],lw=w*72*16/(x1-x0)*0.7,alpha=0.7,solid_capstyle='round')
for (n,x,y) in vias: ax.add_patch(Circle((x,y),0.3,color='k',alpha=0.8))
fp=[(f[1],f[2]) for f in fail]
for p in pads:
    if (p.ref,p.num) in fp: ax.add_patch(Circle((p.x,p.y),0.5,fill=False,color='magenta',lw=1.5))
ax.set_xlim(x0,x1); ax.set_ylim(y1,y0); ax.set_aspect('equal'); ax.grid(alpha=0.2)
plt.savefig(sys.argv[7],dpi=110,bbox_inches='tight')
