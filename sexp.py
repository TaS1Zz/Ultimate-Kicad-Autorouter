import re
TOK=re.compile(r'\s*(\(|\)|"(?:\\.|[^"\\])*"|[^\s()"]+)')
class Sym(str): pass
def parse(s):
    pos=0; stack=[[]]
    for m in TOK.finditer(s):
        t=m.group(1)
        if t=='(': stack.append([])
        elif t==')':
            x=stack.pop(); stack[-1].append(x)
        elif t.startswith('"'): stack[-1].append(t[1:-1].replace('\\"','"').replace('\\\\','\\'))
        else: stack[-1].append(Sym(t))
    return stack[0]
def q(s): return '"'+s.replace('\\','\\\\').replace('"','\\"')+'"'
def dump(x,ind=0):
    if isinstance(x,list):
        simple=all(not isinstance(e,list) for e in x)
        if simple: return '('+' '.join(dump(e) for e in x)+')'
        out='('+' '.join(dump(e) for e in x if not isinstance(e,list))
        for e in x:
            if isinstance(e,list): out+='\n'+'  '*(ind+1)+dump(e,ind+1)
        return out+')'
    if isinstance(x,Sym): return str(x)
    if isinstance(x,(int,float)):
        if isinstance(x,float):
            r=f"{x:.4f}".rstrip('0').rstrip('.'); return r if r not in('-0','') else '0'
        return str(x)
    return q(x)
def find(x,key): return [e for e in x if isinstance(e,list) and e and e[0]==key]
def find1(x,key):
    r=find(x,key); return r[0] if r else None
