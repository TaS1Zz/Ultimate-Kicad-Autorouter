"""Заливки GND на внешних слоях: копии зоны GND со слоя плоскости на F.Cu и B.Cu (заливает KiCad)."""
import re, sys, uuid


def zone_blocks(txt):
    """[(начало, конец, текст)] блоков верхнего уровня (zone …)"""
    out = []
    for m in re.finditer(r'\n\t\(zone\b', txt):
        k = m.start() + 1; depth = 0
        for j in range(k, len(txt)):
            c = txt[j]
            if c == '(': depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0: out.append((k, j + 1, txt[k:j + 1])); break
    return out


def add_zones(txt, net='GND', plane_layers=('In1.Cu', 'In2.Cu'), to=('F.Cu', 'B.Cu')):
    blocks = zone_blocks(txt)
    have = set()
    for _, _, z in blocks:
        n = re.search(r'\(net "([^"]*)"\)', z); l = re.search(r'\(layer "([^"]+)"\)', z)
        if n and l: have.add((n.group(1), l.group(1)))
    src = [z for _, _, z in blocks if '(net "%s")' % net in z and any('(layer "%s")' % l in z for l in plane_layers)]
    if not src: return txt
    z = src[0]; lay = re.search(r'\(layer "([^"]+)"\)', z).group(1)
    new = []
    for l in to:
        if (net, l) in have: continue
        nz = z.replace('(layer "%s")' % lay, '(layer "%s")' % l)
        nz = re.sub(r'\(uuid "[^"]+"\)', '(uuid "%s")' % uuid.uuid4(), nz, count=1)
        nz = re.sub(r'\n\t\t\(filled_polygon.*', '\n\t)', nz, flags=re.S)
        new.append(nz)
    end = blocks[-1][1]
    return txt[:end] + ''.join('\n' + b for b in new) + txt[end:]


if __name__ == '__main__':
    t = open(sys.argv[1], encoding='utf-8').read(); open(sys.argv[2], 'w', encoding='utf-8').write(add_zones(t))
