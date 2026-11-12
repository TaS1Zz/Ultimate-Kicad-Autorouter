import math, collections
def key(x,y): return (round(x,4),round(y,4))

def union_overlaps(S):
    """Отрезки одной цепи/слоя/ширины, лежащие на одной прямой и перекрывающиеся, — в один.
    Такое перекрытие возникает, когда новая ветвь цепи идёт поверх уже проложенной меди;
    KiCad считает конец короткого отрезка висящим (track_dangling)."""
    S=[list(s) for s in S]
    changed=True
    while changed:
        changed=False
        for i in range(len(S)):
            a,b=S[i]; dx,dy=b[0]-a[0],b[1]-a[1]; L=math.hypot(dx,dy)
            if L<1e-9: continue
            ux,uy=dx/L,dy/L
            for j in range(i+1,len(S)):
                c,d=S[j]
                if abs((c[0]-a[0])*uy-(c[1]-a[1])*ux)>1e-4 or abs((d[0]-a[0])*uy-(d[1]-a[1])*ux)>1e-4: continue
                t=sorted([0.0,L]); u=sorted([(c[0]-a[0])*ux+(c[1]-a[1])*uy,(d[0]-a[0])*ux+(d[1]-a[1])*uy])
                if min(t[1],u[1])-max(t[0],u[0])<=1e-6: continue      # только касание — не трогаем (это узел)
                s0,s1=min(t[0],u[0]),max(t[1],u[1])
                S[i]=[key(a[0]+ux*s0,a[1]+uy*s0),key(a[0]+ux*s1,a[1]+uy*s1)]
                del S[j]; changed=True; break
            if changed: break
    return [tuple(s) for s in S]
def merge(tracks, vias):
    groups=collections.defaultdict(list)
    for t in tracks:
        if math.hypot(t[4]-t[2],t[5]-t[3])<1e-6: continue
        groups[(t[0],t[1],round(t[6],4))].append([key(t[2],t[3]),key(t[4],t[5])])
    viaset={key(v[1],v[2]) for v in vias}
    out=[]
    for (n,l,w),segs in groups.items():
        # dedupe
        seen=set(); S=[]
        for a,b in segs:
            k=(a,b) if a<b else (b,a)
            if k in seen: continue
            seen.add(k); S.append(list(k))
        S=union_overlaps(S)
        changed=True
        while changed:
            changed=False
            deg=collections.Counter()
            for a,b in S: deg[a]+=1; deg[b]+=1
            inc=collections.defaultdict(list)
            for i,(a,b) in enumerate(S): inc[a].append(i); inc[b].append(i)
            dead=set()
            for p,ids in inc.items():
                ids=[i for i in ids if i not in dead]
                if len(ids)!=2 or p in viaset: continue
                i,j=ids
                a=S[i][0] if S[i][1]==p else S[i][1]
                c=S[j][0] if S[j][1]==p else S[j][1]
                if a==c: continue
                cr=(p[0]-a[0])*(c[1]-a[1])-(p[1]-a[1])*(c[0]-a[0])
                if abs(cr)>1e-6: continue
                if (p[0]-a[0])*(c[0]-p[0])+(p[1]-a[1])*(c[1]-p[1])<0: continue
                S[i]=[a,c]; dead.add(j); changed=True
                break
            S=[s for k,s in enumerate(S) if k not in dead]
        for a,b in S: out.append((n,l,a[0],a[1],b[0],b[1],w))
    return out
