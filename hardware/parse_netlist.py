#!/usr/bin/env python3
"""Parse an EasyEDA Std schematic .json and reconstruct the electrical netlist.

Connectivity model: a wire (W) joins all its vertices into one node; pins (P, the
'dot' coordinate) and net-label flags (F) attach to whatever shares their coord.
Union-find over rounded coordinates gives the nets."""
import json, sys, re
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else \
    "/Users/bisand/Downloads/SCH_water-temp_2026-06-27.json"
doc = json.load(open(path))
shapes = doc["schematics"][0]["dataStr"]["shape"]

def key(x, y): return (round(float(x)), round(float(y)))

parent = {}
def find(a):
    parent.setdefault(a, a)
    while parent[a] != a:
        parent[a] = parent[parent[a]]; a = parent[a]
    return a
def union(a, b):
    parent[find(a)] = find(b)

pins = []        # (comp_ref, pin_num, pin_name, coordkey)
labels = []      # (netname, coordkey)
nc = set()       # no-connect coords

def parse_lib(s):
    # LIB~x~y~package`...~rot~...~#@$child#@$child...
    head, *rest = s.split("#@$")
    children = rest
    ref, name = None, None
    pinrows = []
    for c in children:
        f = c.split("~")
        if f[0] == "T" and len(f) > 1 and f[1] == "P":
            # designator text: ...~comment~<REF>~...
            try: ref = f[f.index("comment") + 1]
            except ValueError: pass
        if f[0] == "T" and len(f) > 1 and f[1] == "N":
            try: name = f[f.index("comment") + 1]
            except ValueError: pass
        if f[0] == "P":
            # P~show~e~0~<num>~<x>~<y>~<rot>~id~0^^<x>~<y>^^M path...^^...^^<namepart>
            seg = c.split("^^")
            head_f = seg[0].split("~")
            num = head_f[3]
            x, y = head_f[4], head_f[5]
            # pin name: appears in a later segment as ...~<name>~start/end~...
            pname = ""
            for s2 in seg[2:]:
                # e.g. "1~953~-722~0~MISO~end~~~#0000FF"
                parts = s2.split("~")
                for j, p in enumerate(parts):
                    if p in ("start", "end") and j >= 1:
                        cand = parts[j-1]
                        if cand and not re.fullmatch(r"-?\d+(\.\d+)?", cand):
                            pname = cand
                            break
                if pname: break
            pinrows.append((num, pname, key(x, y)))
    return ref, name, pinrows

for s in shapes:
    t = s.split("~")[0]
    if s.startswith("LIB~"):
        ref, name, pinrows = parse_lib(s)
        label = ref or name or "?"
        for num, pname, ck in pinrows:
            pins.append((label, num, pname, ck))
    elif s.startswith("W~"):
        pts = s.split("~")[1].split(" ")
        coords = [key(pts[i], pts[i+1]) for i in range(0, len(pts) - 1, 2)]
        for c in coords[1:]:
            union(coords[0], c)
    elif s.startswith("F~"):
        # F~part_netLabel_xxx~x~y~rot~id~~0^^x~y^^NETNAME~...
        f = s.split("~")
        kind = f[0] + f[1] if False else f[1]
        seg = s.split("^^")
        xy = seg[1].split("~")
        ck = key(xy[0], xy[1])
        netname = seg[2].split("~")[0]
        if "gnD" in f[1]: netname = "GND"
        if f[1].endswith("VCC"): netname = "VCC"
        labels.append((netname, ck))
    elif s.startswith("O~"):
        f = s.split("~")
        nc.add(key(f[1], f[2]))

# union pins and labels into coordinate nodes
for _, _, _, ck in pins: find(ck)
for _, ck in labels: find(ck)

# assign net names
coord_net = {}
for netname, ck in labels:
    coord_net[find(ck)] = netname

nets = defaultdict(list)
for ref, num, pname, ck in pins:
    root = find(ck)
    net = coord_net.get(root, f"_node_{root}")
    nets[net].append(f"{ref}.{num}({pname})")

print("=== NETLIST (net -> connected pins) ===")
for net in sorted(nets, key=lambda n: (n.startswith("_"), n)):
    members = nets[net]
    tag = "  <-- UNNAMED/floating" if net.startswith("_") and len(members) > 1 else ""
    if net.startswith("_") and len(members) == 1:
        continue  # skip single unconnected stubs for now
    print(f"\n{net}{tag}")
    for m in members:
        print(f"    {m}")

print("\n=== single-pin (unconnected) ===")
for net in nets:
    if net.startswith("_") and len(nets[net]) == 1:
        print(f"    {nets[net][0]}")

print(f"\n=== components ({len(set(p[0] for p in pins))}) ===")
for ref in sorted(set(p[0] for p in pins)):
    print("   ", ref)
