#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Regenerates docs/assets/social-preview.png — the card GitHub renders wherever
# the repository is linked.
#
#   python3 scripts/gen-social-preview.py
#
# It draws the s-plane pole map of NT-33A flight condition 1. The five
# eigenvalues below are the ones galata computes; they come from
#
#   galata run examples/nt33a-trim-and-linearise/study.yaml
#
# and their modal metrics are locked by tests/integration/test_example_studies.cpp
# and validated against NASA CR-2144 by tests/validation/test_nt33a_modes.cpp.
# If those tests ever move, this picture is wrong and must be regenerated.
#
# Requires rsvg-convert (librsvg) to rasterise. The SVG is written next to the
# PNG so the vector original stays reviewable.

W, H = 1280, 640
LONG, LAT = "#5BC8F5", "#FFB454"
# Poles from: galata run examples/nt33a-trim-and-linearise/study.yaml
POLES = [("short period", -0.9920, 1.2490, LONG),
         ("phugoid",      -0.0163, 0.1706, LONG),
         ("Dutch roll",   -0.0681, 1.1273, LAT),
         ("roll subsidence", -2.1992, 0.0, LAT),
         ("spiral",       -0.0319, 0.0,    LAT)]

BG, PANEL, GRID, AXIS = "#0B1016", "#0E141C", "#1B2733", "#3F5165"
FG, DIM, MUTE, RHP, RHPL = "#E6EDF3", "#93A1AF", "#5E6B78", "#2B171D", "#9E6874"

o = []; A = o.append
A(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
A('<defs><style>'
  '.s{font-family:"Helvetica Neue",Helvetica,Arial,sans-serif}'
  '.m{font-family:"SF Mono",Menlo,Consolas,monospace}'
  '</style>')

class P:
    def __init__(s, x0, x1, y0, y1, rmin, rmax, imax, cid):
        s.x0,s.x1,s.y0,s.y1,s.cid = x0,x1,y0,y1,cid
        s.rmin,s.rmax,s.imax = rmin,rmax,imax
        s.sx = (x1-x0)/(rmax-rmin); s.sy = (y1-y0)/(2*imax)
        s.ox = x0 + (0-rmin)*s.sx; s.oy = y0 + imax*s.sy
        A(f'<clipPath id="{cid}"><rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}"/></clipPath>')
    def X(s,r): return s.ox + r*s.sx
    def Y(s,i): return s.oy - i*s.sy

PA = P(556, 1216,  88, 380, -2.45, 0.25, 1.5,  "ca")   # overview
PB = P(556, 1216, 412, 548, -0.070, 0.010, 0.30, "cb")   # magnified about the origin
A('</defs>')
A(f'<rect width="{W}" height="{H}" fill="{BG}"/>')

# ------------------------------- left column -------------------------------
A(f'<text class="s" x="64" y="150" fill="{FG}" font-size="78" font-weight="600" letter-spacing="-1.5">galata</text>')
A(f'<rect x="66" y="176" width="54" height="4" fill="{LONG}"/>')
A(f'<text class="s" x="64" y="226" fill="{FG}" font-size="22" font-weight="500">Flight dynamics and control-law design</text>')
for i, t in enumerate(["Trim a nonlinear aircraft, linearise about",
                       "the trim, and read off a modal table whose",
                       "labels come from eigenvector participation,",
                       "not from frequency order."]):
    A(f'<text class="s" x="64" y="{268+i*26}" fill="{DIM}" font-size="18">{t}</text>')

A(f'<rect x="64" y="396" width="4" height="84" fill="{LAT}"/>')
A(f'<text class="s" x="86" y="418" fill="{FG}" font-size="19" font-weight="600">NASA CR-2144, flight condition 1</text>')
A(f'<text class="s" x="86" y="446" fill="{DIM}" font-size="18">dimensional derivatives to <tspan fill="{FG}" font-weight="600">0.26%</tspan></text>')
A(f'<text class="s" x="86" y="471" fill="{DIM}" font-size="18">all five classical modes to <tspan fill="{FG}" font-weight="600">1.0%</tspan></text>')
A(f'<text class="s" x="64" y="536" fill="{MUTE}" font-size="15" letter-spacing="0.5">C++20  ·  strict SI  ·  deterministic  ·  Apache-2.0</text>')

def cross(x, y, c, sz=8.5, w=2.6):
    return (f'<g stroke="{c}" stroke-width="{w}" stroke-linecap="round">'
            f'<line x1="{x-sz:.1f}" y1="{y-sz:.1f}" x2="{x+sz:.1f}" y2="{y+sz:.1f}"/>'
            f'<line x1="{x-sz:.1f}" y1="{y+sz:.1f}" x2="{x+sz:.1f}" y2="{y-sz:.1f}"/></g>')

def frame(p, rgrid, igrid, sz, wd):
    A(f'<rect x="{p.x0}" y="{p.y0}" width="{p.x1-p.x0}" height="{p.y1-p.y0}" '
      f'fill="{PANEL}" stroke="{GRID}" stroke-width="1" rx="3"/>')
    A(f'<g clip-path="url(#{p.cid})">')
    A(f'<rect x="{p.X(0):.1f}" y="{p.y0}" width="{p.x1-p.X(0):.1f}" height="{p.y1-p.y0}" fill="{RHP}"/>')
    for r in rgrid:
        A(f'<line x1="{p.X(r):.1f}" y1="{p.y0}" x2="{p.X(r):.1f}" y2="{p.y1}" stroke="{GRID}" stroke-width="1"/>')
    for i in igrid:
        for s_ in (i, -i):
            A(f'<line x1="{p.x0}" y1="{p.Y(s_):.1f}" x2="{p.x1}" y2="{p.Y(s_):.1f}" stroke="{GRID}" stroke-width="1"/>')
    A(f'<line x1="{p.x0}" y1="{p.oy:.1f}" x2="{p.x1}" y2="{p.oy:.1f}" stroke="{AXIS}" stroke-width="1.4"/>')
    A(f'<line x1="{p.X(0):.1f}" y1="{p.y0}" x2="{p.X(0):.1f}" y2="{p.y1}" stroke="#7A4E59" stroke-width="1.4"/>')
    for _, re_, im_, c in POLES:
        if not (p.rmin <= re_ <= p.rmax and abs(im_) <= p.imax):
            continue
        A(cross(p.X(re_), p.Y(im_), c, sz, wd))
        if im_:
            A(cross(p.X(re_), p.Y(-im_), c, sz, wd))
    A('</g>')

def lab(x, y, t, c, anchor="start", size=16, weight="500"):
    A(f'<text class="s" x="{x:.1f}" y="{y:.1f}" fill="{c}" font-size="{size}" '
      f'font-weight="{weight}" text-anchor="{anchor}">{t}</text>')

def leader(x1, y1, x2, y2, c):
    A(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{c}" stroke-width="1.2" opacity="0.55"/>')

# ------------------------------- panel A -----------------------------------
frame(PA, [-2.0, -1.5, -1.0, -0.5], [0.5, 1.0], 8.5, 2.6)
for r in [-2.0, -1.5, -1.0, -0.5]:
    A(f'<text class="m" x="{PA.X(r):.1f}" y="{PA.oy+17:.1f}" fill="{MUTE}" font-size="12" text-anchor="middle">{r:g}</text>')
for i in [0.5, 1.0]:
    for s_ in (i, -i):
        A(f'<text class="m" x="{PA.x0+7}" y="{PA.Y(s_)-5:.1f}" fill="{MUTE}" font-size="12">{s_:+g}j</text>')
A(f'<text class="s" x="{PA.x1-6}" y="{PA.y0+18}" fill="{RHPL}" font-size="12" text-anchor="end">unstable</text>')

lab(PA.X(-0.992)-16, PA.Y(1.249)-13, "short period", LONG, "end")
leader(PA.X(-0.992)-12, PA.Y(1.249)-8, PA.X(-0.992)-4, PA.Y(1.249)-3, LONG)
lab(PA.X(-0.0681)-18, PA.Y(1.1273)-13, "Dutch roll", LAT, "end")
leader(PA.X(-0.0681)-14, PA.Y(1.1273)-8, PA.X(-0.0681)-5, PA.Y(1.1273)-3, LAT)
lab(PA.X(-2.1992), PA.Y(0)-22, "roll subsidence", LAT, "middle")
leader(PA.X(-2.1992), PA.Y(0)-18, PA.X(-2.1992), PA.Y(0)-11, LAT)

# the region panel B magnifies
bx0, bx1 = PA.X(PB.rmin), PA.X(PB.rmax)
by0, by1 = PA.Y(PB.imax), PA.Y(-PB.imax)
A(f'<rect x="{bx0:.1f}" y="{by0:.1f}" width="{bx1-bx0:.1f}" height="{by1-by0:.1f}" '
  f'fill="none" stroke="{MUTE}" stroke-width="1" stroke-dasharray="3 2"/>')
for xa, xb in ((bx0, PB.x0), (bx1, PB.x1)):
    A(f'<line x1="{xa:.1f}" y1="{by1:.1f}" x2="{xb}" y2="{PB.y0}" stroke="{MUTE}" '
      f'stroke-width="1" stroke-dasharray="3 2" opacity="0.35"/>')

# legend, in panel A's empty lower-left
A(f'<g transform="translate({PA.x0+26},{PA.y1-20})">')
A(cross(0, 0, LONG, 6.5, 2.2)); A(f'<text class="s" x="15" y="5" fill="{DIM}" font-size="14">longitudinal</text>')
A(cross(122, 0, LAT, 6.5, 2.2)); A(f'<text class="s" x="137" y="5" fill="{DIM}" font-size="14">lateral–directional</text>')
A('</g>')

# ------------------------------- panel B -----------------------------------
frame(PB, [-0.06, -0.04, -0.02], [0.2], 8.0, 2.5)
for r in [-0.06, -0.04, -0.02]:
    A(f'<text class="m" x="{PB.X(r):.1f}" y="{PB.oy+17:.1f}" fill="{MUTE}" font-size="12" text-anchor="middle">{r:g}</text>')
A(f'<text class="s" x="{PB.x0+12}" y="{PB.y0+20}" fill="{DIM}" font-size="12" letter-spacing="0.8">THE SLOW MODES, MAGNIFIED ABOUT THE ORIGIN</text>')
lab(PB.X(-0.0163)-16, PB.Y(0.1706)+5, "phugoid", LONG, "end")
leader(PB.X(-0.0163)-12, PB.Y(0.1706), PB.X(-0.0163)-5, PB.Y(0.1706), LONG)
lab(PB.X(-0.0319), PB.Y(0)+26, "spiral", LAT, "middle")
leader(PB.X(-0.0319), PB.Y(0)+11, PB.X(-0.0319), PB.Y(0)+17, LAT)

A(f'<text class="s" x="{PB.x1}" y="{PB.y1+26}" fill="{MUTE}" font-size="13" text-anchor="end">'
  f's-plane, units of 1/s  ·  NT-33A at sea level, M = 0.204  ·  poles as computed by galata</text>')
A('</svg>')
import pathlib, subprocess
root = pathlib.Path(__file__).resolve().parent.parent
out = root / "docs" / "assets"
out.mkdir(parents=True, exist_ok=True)
svg, png = out / "social-preview.svg", out / "social-preview.png"
svg.write_text("\n".join(o))
subprocess.run(["rsvg-convert", "-w", str(W), "-h", str(H), "-o", str(png), str(svg)], check=True)
print(f"wrote {svg.relative_to(root)} and {png.relative_to(root)}")
