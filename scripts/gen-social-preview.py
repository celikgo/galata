#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Regenerates docs/assets/social-preview.png — the card GitHub renders wherever
# this repository is linked.
#
#   ./build/dev/tools/social/galata-modal-map > docs/assets/modal-map.json
#   python3 scripts/gen-social-preview.py
#
# THE NUMBERS ARE NOT IN THIS FILE. Every pole drawn here is read from
# docs/assets/modal-map.json, which is emitted by tools/social/main.cpp running
# the same trim-and-linearise chain the validation tier gates. The picture
# claims that galata computes these poles and labels them by eigenvector
# participation; drawing it from hand-typed coordinates would be making that
# claim without evidence, and it would go stale the first time the model moved.
#
# The MODE NAMES are read from the same file for the same reason. They are
# assigned by analyze::analyze_modes from participation factors, not by this
# script and not by frequency order — which is the repository's distinguishing
# feature and the thing the picture exists to show.
#
# Requires rsvg-convert (librsvg) to rasterise. The SVG is written next to the
# PNG so the vector original stays reviewable in a diff.

import json
import math
import pathlib
import subprocess
import sys

W, H = 1280, 640
LONG, LAT = "#5BC8F5", "#FFB454"
BG, PANEL, GRID, GRIDL = "#0B1016", "#0E141C", "#1B2733", "#233243"
AXIS, FG, DIM, MUTE = "#3F5165", "#E6EDF3", "#93A1AF", "#5E6B78"
RHP, RHPL = "#2B171D", "#9E6874"

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "assets"
DATA = OUT / "modal-map.json"

if not DATA.exists():
    sys.exit(
        f"{DATA.relative_to(ROOT)} is missing.\n"
        "Build and run the emitter first, so this picture is a run and not a drawing:\n"
        "  cmake --build --preset dev --target galata-modal-map\n"
        "  ./build/dev/tools/social/galata-modal-map > docs/assets/modal-map.json"
    )

run = json.loads(DATA.read_text())
MODES = run["modes"]
COLOUR = {"longitudinal": LONG, "lateral": LAT}


def pole(name):
    for m in MODES:
        if m["label"] == name:
            return m
    sys.exit(f"{DATA.name} has no mode labelled {name!r}: the classifier's output changed, "
             "so this picture's annotations are no longer true. Fix the labels here.")


o = []
A = o.append
A(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
A('<defs><style>'
  '.s{font-family:"Helvetica Neue",Helvetica,Arial,sans-serif}'
  '.m{font-family:"SF Mono",Menlo,Consolas,monospace}'
  '</style>')


class Plot:
    """An s-plane panel with EQUAL SCALE ON BOTH AXES.

    That is not a stylistic choice. Constant-omega_n loci are circles about the
    origin and constant-zeta loci are rays through it; under unequal scaling the
    circles become ellipses and the rays no longer read as constant damping, so
    the grid would be decorative rather than something you can measure against.
    """

    def __init__(s, x0, y0, size, r0, r1, cid):
        s.x0, s.y0, s.x1, s.y1, s.cid = x0, y0, x0 + size, y0 + size, cid
        s.r0, s.r1 = r0, r1
        s.k = size / (r1 - r0)               # px per unit, both axes
        s.imax = (size / s.k) / 2.0
        s.ox, s.oy = x0 + (0 - r0) * s.k, y0 + size / 2.0
        A(f'<clipPath id="{cid}"><rect x="{x0}" y="{y0}" width="{size}" height="{size}"/></clipPath>')

    def X(s, re):
        return s.ox + re * s.k

    def Y(s, im):
        return s.oy - im * s.k


PA = Plot(64, 200, 400, -2.75, 0.15, "ca")     # the whole map
PB = Plot(504, 200, 400, -0.31, 0.07, "cb")    # magnified about the origin
A('</defs>')
A(f'<rect width="{W}" height="{H}" fill="{BG}"/>')


def cross(x, y, c, sz=8.0, w=2.5):
    return (f'<g stroke="{c}" stroke-width="{w}" stroke-linecap="round">'
            f'<line x1="{x-sz:.1f}" y1="{y-sz:.1f}" x2="{x+sz:.1f}" y2="{y+sz:.1f}"/>'
            f'<line x1="{x-sz:.1f}" y1="{y+sz:.1f}" x2="{x+sz:.1f}" y2="{y-sz:.1f}"/></g>')


def text(x, y, t, c, size=15, anchor="start", weight="500", cls="s", extra=""):
    A(f'<text class="{cls}" x="{x:.1f}" y="{y:.1f}" fill="{c}" font-size="{size}" '
      f'font-weight="{weight}" text-anchor="{anchor}"{extra}>{t}</text>')


def leader(x1, y1, x2, y2, c):
    A(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{c}" '
      f'stroke-width="1.1" opacity="0.6"/>')


def grid(p, wn_arcs, zetas, wn_fmt="{:g}", zeta_label_frac=1.0):
    """Constant-omega_n circles and constant-zeta rays, both centred on the origin."""
    A(f'<rect x="{p.x0}" y="{p.y0}" width="{p.x1-p.x0}" height="{p.y1-p.y0}" '
      f'fill="{PANEL}" stroke="{GRID}" stroke-width="1" rx="3"/>')
    A(f'<g clip-path="url(#{p.cid})">')
    A(f'<rect x="{p.X(0):.1f}" y="{p.y0}" width="{p.x1-p.X(0):.1f}" '
      f'height="{p.y1-p.y0}" fill="{RHP}"/>')

    # Constant omega_n: half-circles in the left half plane. Drawn as polylines
    # rather than SVG arcs so there is no sweep-flag to get backwards.
    for wn in wn_arcs:
        r = wn * p.k
        pts = " ".join(f"{p.ox + r*math.cos(t):.1f},{p.oy - r*math.sin(t):.1f}"
                       for t in [math.pi/2 + i*math.pi/72 for i in range(73)])
        A(f'<polyline points="{pts}" fill="none" stroke="{GRID}" stroke-width="1"/>')
        text(p.X(-wn) + 4, p.oy - 5, wn_fmt.format(wn), MUTE, 10.5, cls="m", weight="400")

    # Constant zeta: rays from the origin through omega_n(-zeta +/- j sqrt(1-zeta^2)).
    reach = (p.x1 - p.x0) * 1.5 / p.k
    span = p.y1 - p.oy - 20
    for i, z in enumerate(zetas):
        dx, dy = -z, math.sqrt(1.0 - z * z)
        for sgn in (1, -1):
            A(f'<line x1="{p.ox:.1f}" y1="{p.oy:.1f}" '
              f'x2="{p.X(dx*reach):.1f}" y2="{p.Y(sgn*dy*reach):.1f}" '
              f'stroke="{GRIDL}" stroke-width="1" stroke-dasharray="2 3"/>')
        # Label on the LOWER ray: the upper half of every panel is where the mode
        # annotations sit. The depth is STAGGERED across the rays — in the
        # magnified panel every ray is nearly vertical, so labelling them all at
        # one height piles four labels into 30 px of width.
        frac = zeta_label_frac * (0.52 + 0.48 * i / max(1, len(zetas) - 1))
        depth = span * frac
        t = depth / (dy * p.k)
        lx, ly = p.X(dx * t), p.oy + depth + 11
        if p.x0 + 14 < lx < p.x1 - 14:
            text(lx, ly, f"ζ {z:g}", MUTE, 10.5, "middle", "400", cls="m")

    A(f'<line x1="{p.x0}" y1="{p.oy:.1f}" x2="{p.x1}" y2="{p.oy:.1f}" '
      f'stroke="{AXIS}" stroke-width="1.3"/>')
    A(f'<line x1="{p.X(0):.1f}" y1="{p.y0}" x2="{p.X(0):.1f}" y2="{p.y1}" '
      f'stroke="#7A4E59" stroke-width="1.3"/>')


def draw_poles(p, sz=8.0, w=2.5):
    for m in MODES:
        re, im = m["re"], m["im"]
        if not (p.r0 <= re <= p.r1 and abs(im) <= p.imax):
            continue
        c = COLOUR[m["axis"]]
        A(cross(p.X(re), p.Y(im), c, sz, w))
        if m["oscillatory"]:
            A(cross(p.X(re), p.Y(-im), c, sz, w))


# ------------------------------- masthead ----------------------------------
text(64, 92, "galata", FG, 58, weight="600", extra=' letter-spacing="-1.5"')
A(f'<rect x="66" y="112" width="52" height="4" fill="{LONG}"/>')
text(64, 152, "Flight dynamics, control-law design and simulation", FG, 21, weight="500")
text(64, 178, "Trim a nonlinear aircraft, linearise about the trim, and read off a modal table "
              "whose labels come from eigenvector participation.", DIM, 15.5)

text(1216, 92, "NASA CR-2144, flight condition 1", FG, 17, "end", "600")
text(1216, 116, "dimensional derivatives to 0.26%", DIM, 15, "end")
text(1216, 138, "all five classical modes to 1.0%", DIM, 15, "end")
A(f'<rect x="1032" y="152" width="184" height="3" fill="{LAT}"/>')

# ------------------------------- panel A -----------------------------------
grid(PA, [0.5, 1.0, 1.5, 2.0, 2.5], [0.2, 0.4, 0.6, 0.8])
draw_poles(PA)

sp, dr, rs = pole("short period"), pole("Dutch roll"), pole("roll subsidence")
text(PA.X(sp["re"]) - 15, PA.Y(sp["im"]) - 11, "short period", LONG, 15, "end", "600")
leader(PA.X(sp["re"]) - 12, PA.Y(sp["im"]) - 7, PA.X(sp["re"]) - 4, PA.Y(sp["im"]) - 3, LONG)
text(PA.X(dr["re"]) - 15, PA.Y(dr["im"]) - 11, "Dutch roll", LAT, 15, "end", "600")
leader(PA.X(dr["re"]) - 12, PA.Y(dr["im"]) - 7, PA.X(dr["re"]) - 4, PA.Y(dr["im"]) - 3, LAT)
text(PA.X(rs["re"]), PA.Y(0) - 25, "roll subsidence", LAT, 15, "middle", "600")
leader(PA.X(rs["re"]), PA.Y(0) - 21, PA.X(rs["re"]), PA.Y(0) - 12, LAT)

text(PA.x1 - 8, PA.y0 + 15, "unstable", RHPL, 11.5, "end", "400")
text(PA.x0 + 12, PA.y0 + 20, "THE WHOLE MAP", MUTE, 11, weight="600",
     extra=' letter-spacing="1.2"')
A('</g>')

# The region panel B magnifies, marked on panel A and tied to it.
bx0, bx1 = PA.X(PB.r0), PA.X(PB.r1)
by0, by1 = PA.Y(PB.imax), PA.Y(-PB.imax)
A(f'<rect x="{bx0:.1f}" y="{by0:.1f}" width="{bx1-bx0:.1f}" height="{by1-by0:.1f}" '
  f'fill="none" stroke="{MUTE}" stroke-width="1" stroke-dasharray="3 2"/>')
for ya, yb in ((by0, PB.y0), (by1, PB.y1)):
    A(f'<line x1="{bx1:.1f}" y1="{ya:.1f}" x2="{PB.x0}" y2="{yb}" stroke="{MUTE}" '
      f'stroke-width="1" stroke-dasharray="3 2" opacity="0.35"/>')

# ------------------------------- panel B -----------------------------------
grid(PB, [0.10, 0.15, 0.20, 0.25, 0.30], [0.1, 0.2, 0.4, 0.6], wn_fmt="{:.2f}")
draw_poles(PB, 8.0, 2.5)

ph, sr = pole("phugoid"), pole("spiral")
text(PB.X(ph["re"]) - 15, PB.Y(ph["im"]) + 5, "phugoid", LONG, 15, "end", "600")
leader(PB.X(ph["re"]) - 12, PB.Y(ph["im"]), PB.X(ph["re"]) - 5, PB.Y(ph["im"]), LONG)
text(PB.X(sr["re"]), PB.Y(0) + 27, "spiral", LAT, 15, "middle", "600")
leader(PB.X(sr["re"]), PB.Y(0) + 12, PB.X(sr["re"]), PB.Y(0) + 18, LAT)
text(PB.x0 + 12, PB.y0 + 20,
     f"MAGNIFIED  ·  {PB.k / PA.k:.1f}×", MUTE, 11, weight="600",
     extra=' letter-spacing="1.2"')
A('</g>')

# ------------------------------ right column -------------------------------
CX = 944
text(CX, 214, "CLASSIFIED BY PARTICIPATION", DIM, 11.5, weight="600",
     extra=' letter-spacing="1"')
A(f'<line x1="{CX}" y1="226" x2="1216" y2="226" stroke="{GRID}" stroke-width="1"/>')
text(CX, 246, "mode", MUTE, 11.5, weight="500")
text(1150, 246, "ω", MUTE, 11.5, "end", "500")
text(1216, 246, "ζ", MUTE, 11.5, "end", "500")

order = ["short period", "phugoid", "Dutch roll", "roll subsidence", "spiral"]
y = 272
for name in order:
    m = pole(name)
    c = COLOUR[m["axis"]]
    A(cross(CX + 6, y - 5, c, 5.5, 2.0))
    text(CX + 20, y, name, FG, 14, weight="500")
    text(1150, y, f'{m["omega_n"]:.3f}', DIM, 13, "end", "400", cls="m")
    text(1216, y, "—" if not m["oscillatory"] else f'{m["zeta"]:.4f}', DIM, 13, "end", "400",
         cls="m")
    y += 27

A(f'<line x1="{CX}" y1="{y-8}" x2="1216" y2="{y-8}" stroke="{GRID}" stroke-width="1"/>')
text(CX, y + 14, "For a real root ω is 1/T, and it has no ζ.", MUTE, 12.5, weight="400")
text(CX, y + 33, "Labels are assigned from eigenvector", MUTE, 12.5, weight="400")
text(CX, y + 50, "participation, not from frequency order.", MUTE, 12.5, weight="400")

A(f'<rect x="{CX}" y="{y+72}" width="4" height="56" fill="{LONG}"/>')
text(CX + 16, y + 90, "C++20  ·  strict SI", FG, 14, weight="600")
text(CX + 16, y + 110, "deterministic  ·  Apache-2.0", DIM, 13.5)

# --------------------------------- footer ----------------------------------
cond = run["condition"]
text(64, 628, f's-plane, units of 1/s  ·  NT-33A at sea level, M = {cond["mach"]:.3f}, '
              f'trimmed to {cond["alpha_deg"]:.2f}° alpha  ·  '
              f'poles as computed by galata {run["version"]}', MUTE, 13, weight="400")
A('</svg>')

OUT.mkdir(parents=True, exist_ok=True)
svg, png = OUT / "social-preview.svg", OUT / "social-preview.png"
svg.write_text("\n".join(o))
subprocess.run(["rsvg-convert", "-w", str(W), "-h", str(H), "-o", str(png), str(svg)], check=True)
print(f"wrote {svg.relative_to(ROOT)} and {png.relative_to(ROOT)} "
      f"from {DATA.relative_to(ROOT)} ({len(MODES)} modes)")
