import math, collections
from shapely.geometry import Point, LineString, box
from board import CU

def padpoly(p):
    w,h=(p.w,p.h) if round(p.ang)%180==0 else (p.h,p.w)
    if p.shape=='circle': return Point(p.x,p.y).buffer(p.w/2,24)
    if p.shape=='oval':
        r=min(w,h)/2; l=(max(w,h)-min(w,h))/2
        if l<=0: return Point(p.x,p.y).buffer(r,24)
        return (LineString([(p.x-l,p.y),(p.x+l,p.y)]) if w>=h else LineString([(p.x,p.y-l),(p.x,p.y+l)])).buffer(r,24)
    return box(p.x-w/2,p.y-h/2,p.x+w/2,p.y+h/2)

def net_components(net, tracks, vias, pads):
    its=[]
    for p in pads:
        if p.net!=net or p.kind=='np_thru_hole': continue
        g=padpoly(p); oid=('pad',p.ref,p.num,p.x,p.y)
        for l in p.layers: its.append((CU.index(l),g,oid,True))
    for k,t in enumerate(tracks):
        if t[0]!=net: continue
        g=LineString([(t[2],t[3]),(t[4],t[5])]).buffer(t[6]/2,8) if math.hypot(t[4]-t[2],t[5]-t[3])>0 else Point(t[2],t[3]).buffer(t[6]/2,8)
        its.append((t[1],g,('t',k),False))
    for k,v in enumerate(vias):
        if v[0]!=net: continue
        g=Point(v[1],v[2]).buffer(0.3,16)
        for l in range(4): its.append((l,g,('v',k),False))
    N=len(its); par=list(range(N))
    def f(a):
        while par[a]!=a: par[a]=par[par[a]]; a=par[a]
        return a
    byl=collections.defaultdict(list)
    for i,it in enumerate(its): byl[it[0]].append(i)
    for l,idx in byl.items():
        for a in range(len(idx)):
            for b in range(a+1,len(idx)):
                i,j=idx[a],idx[b]
                if its[i][1].distance(its[j][1])<1e-6: par[f(i)]=f(j)
    if net=='GND':
        idx=byl.get(1,[])
        for i in idx[1:]: par[f(i)]=f(idx[0])
    byo=collections.defaultdict(list)
    for i,it in enumerate(its): byo[it[2]].append(i)
    for v in byo.values():
        for i in v[1:]: par[f(i)]=f(v[0])
    return len({f(i) for i in range(N) if its[i][3]})

def postprocess(tracks, vias, pads):
    tracks=[tuple(t) for t in tracks]; vias=[tuple(v) for v in vias]
    # 1) merge close same-net vias (hole-to-hole)
    i=0
    while i<len(vias):
        j=i+1
        while j<len(vias):
            a,b=vias[i],vias[j]
            if a[0]==b[0] and math.hypot(a[1]-b[1],a[2]-b[2])<0.56:
                lays=set(t[1] for t in tracks if t[0]==b[0] and (math.hypot(t[2]-b[1],t[3]-b[2])<0.31 or math.hypot(t[4]-b[1],t[5]-b[2])<0.31))
                if b[0]=='GND' and not lays: lays={3}
                nt=tracks+[(b[0],l,b[1],b[2],a[1],a[2],0.25) for l in lays]
                nv=vias[:j]+vias[j+1:]
                if net_components(b[0],nt,nv,pads)==1:
                    tracks,vias=nt,nv; continue
            j+=1
        i+=1
    # 2) drop vias that touch tracks on fewer than two layers, if connectivity survives
    k=0
    while k<len(vias):
        v=vias[k]
        if v[0]!='GND':
            used=set(t[1] for t in tracks if t[0]==v[0] and (math.hypot(t[2]-v[1],t[3]-v[2])<0.3 or math.hypot(t[4]-v[1],t[5]-v[2])<0.3))
            if len(used)<2:
                nv=vias[:k]+vias[k+1:]
                if net_components(v[0],tracks,nv,pads)==1:
                    vias=nv; continue
        k+=1
    # 3) висящие хвосты: отрезок, у которого один конец ни к чему своей цепи не касается
    #    (остаток fanout, когда цепь подключилась к площадке напрямую) — удалить; затем снова
    #    убрать переходы, оставшиеся на одном слое. Повторять, пока есть что убирать.
    changed=True
    while changed:
        changed=False
        tracks,removed=trim_dangling(tracks,vias,pads)
        if removed: changed=True
        k=0
        while k<len(vias):
            v=vias[k]
            if v[0]!='GND':
                used=set(t[1] for t in tracks if t[0]==v[0] and (math.hypot(t[2]-v[1],t[3]-v[2])<0.3 or math.hypot(t[4]-v[1],t[5]-v[2])<0.3))
                touches_pad=any(p.net==v[0] and padpoly(p).distance(Point(v[1],v[2]))<0.3 for p in pads)
                if len(used)<2 and not touches_pad:
                    nv=vias[:k]+vias[k+1:]
                    if net_components(v[0],tracks,nv,pads)==1 or len(used)==0:
                        vias=nv; changed=True; continue
            k+=1
    return tracks, vias

def trim_dangling(tracks, vias, pads):
    """Висящий конец отрезка: если отрезок проходит через площадку своей цепи — укоротить его до этой
    площадки (по той же прямой, угол не меняется); иначе удалить, но только если связность не рвётся."""
    tracks=list(tracks); changed=0; skip=set()
    for _ in range(2*len(tracks)+1):
        k,end=_find_dangling(tracks,vias,pads,skip)
        if k is None: break
        t=tracks[k]; seg=LineString([(t[2],t[3]),(t[4],t[5])])
        free=(t[2],t[3]) if end==0 else (t[4],t[5]); other=(t[4],t[5]) if end==0 else (t[2],t[3])
        # точки, где к отрезку что-то подключено: площадки, концы других трасс (Т-примыкание), переходы
        att=[]
        for p in pads:
            if p.net==t[0] and CU[t[1]] in p.layers and padpoly(p).intersects(seg): att.append((p.x,p.y))
        for j,u in enumerate(tracks):
            if j==k or u[0]!=t[0] or u[1]!=t[1]: continue
            for e in ((u[2],u[3]),(u[4],u[5])):
                if seg.distance(Point(e))<=t[6]/2+1e-6: att.append(e)
        for v in vias:
            if v[0]==t[0] and seg.distance(Point(v[1],v[2]))<0.3: att.append((v[1],v[2]))
        sf=seg.project(Point(free)); best=None
        for a in att:
            s=seg.project(Point(a))
            if abs(s-sf)<1e-6: continue
            if best is None or abs(s-sf)<abs(best-sf): best=s
        if best is not None:
            q=seg.interpolate(best)
            if math.hypot(q.x-other[0],q.y-other[1])>1e-6:
                # укоротить до ближайшего подключения: прямая та же, угол не меняется
                tracks[k]=(t[0],t[1],other[0],other[1],q.x,q.y,t[6]); changed+=1; continue
        nt=tracks[:k]+tracks[k+1:]
        if net_components(t[0],nt,vias,pads)<=net_components(t[0],tracks,vias,pads):
            tracks=nt; changed+=1; continue
        skip.add(tuple(t))          # убрать нельзя — оставить как есть
    return tracks, changed

def _find_dangling(tracks, vias, pads, skip=()):
    padg=collections.defaultdict(list)
    for p in pads:
        if p.net and p.kind!='np_thru_hole':
            g=padpoly(p)
            for l in p.layers: padg[(p.net,CU.index(l))].append(g)
    vpos=collections.defaultdict(list)
    for v in vias: vpos[v[0]].append((v[1],v[2]))
    bynet=collections.defaultdict(list)
    for k,t in enumerate(tracks): bynet[t[0]].append(k)
    for n,ks in bynet.items():
        for k in ks:
            t=tracks[k]
            if tuple(t) in skip: continue
            for e,(x,y) in enumerate(((t[2],t[3]),(t[4],t[5]))):
                pt=Point(x,y)
                if any(g.distance(pt)<1e-6 for g in padg[(n,t[1])]): continue
                if any(math.hypot(x-a,y-b)<0.3 for (a,b) in vpos[n]): continue
                touch=False
                for j in ks:
                    if j==k: continue
                    u=tracks[j]
                    if u[1]!=t[1]: continue
                    if LineString([(u[2],u[3]),(u[4],u[5])]).distance(pt)<=u[6]/2+1e-6: touch=True; break
                if not touch: return k,e
    return None,None
