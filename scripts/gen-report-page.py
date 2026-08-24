#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Renders docs/reports/nt33a-fc1.html — one self-contained page for one
# aircraft at one flight condition.
#
#   ./build/dev/tools/report/galata-report-data > docs/assets/nt33a-fc1-run.json
#   python3 scripts/gen-report-page.py              # write
#   python3 scripts/gen-report-page.py --check      # diff only, for CI
#
# THE NUMBERS ARE NOT IN THIS FILE, and neither are the mode names. Every
# figure, every table cell and every pole drawn here is read from
# docs/assets/nt33a-fc1-run.json, which tools/report/main.cpp emits by running
# the same chain the validation tier gates. This script knows how to draw an
# s-plane and a Bode plot; it does not know what an NT-33A is.
#
# That is the same argument scripts/gen-social-preview.py makes for the social
# card, and it matters more here. The card is an advertisement; this page is
# the artefact a flight-dynamics engineer would actually read, and a page whose
# numbers were transcribed would be asserting that galata computes them without
# galata ever having computed them.
#
# WHY THIS IS A SEPARATE STEP FROM THE EMITTER. The JSON is compared
# NUMERICALLY across platforms with a stated tolerance (scripts/gen-report.sh
# --check), because it is downstream of a central difference and ADR-0004 does
# not claim a cross-platform bound for such values. The HTML is compared as
# TEXT, because it is a pure function of the JSON: given the same JSON, this
# script emits the same bytes on every platform. Two artefacts, two kinds of
# gate, each the right one for what it is.
#
# NO JAVASCRIPT, NO NETWORK. The page must open from a file:// URL on a
# disconnected laptop and render identically. The light/dark switch is a
# checkbox and a :has() selector; printing forces the light palette, because a
# page printed dark is a page nobody can read.
#
# THE PALETTE is the Twitter Dim scheme this author's projects converge on,
# taken from blaeu-lib's packages/core/src/theme/themes/twitter.ts rather than
# from X's own values. That file applies three documented contrast corrections
# and this page needs all of them: axis labels and gridlines have to be legible,
# and X's raw muted greys are not. Its light theme is what the light variant and
# the print stylesheet use, including the deepened #C2410C in place of the snap
# yellow, which is 1.43:1 on white and therefore invisible.
#
# One token is this page's own rather than the theme's: --grid, the plot
# gridlines. On the dark ground it equals the theme's border grey; on white it
# is deepened, because the border grey that reads correctly around a table cell
# disappears behind a curve when the page is projected.

import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
DATA = ROOT / "docs" / "assets" / "nt33a-fc1-run.json"
OUT = ROOT / "docs" / "reports" / "nt33a-fc1.html"

REPO = "https://github.com/celikgo/galata"
BLOB = f"{REPO}/blob/main/"
SITE = "https://celikgo.github.io/galata/"


# --------------------------------------------------------------------------
# Formatting. A quantity the run reports as absent is a dash, never a zero.
# --------------------------------------------------------------------------

def esc(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def num(value, figures=4):
    """A number for a table cell."""
    if value is None:
        return "&mdash;"
    if isinstance(value, str):
        return "infinite" if value == "infinite" else esc(value)
    if isinstance(value, bool):
        return "yes" if value else "no"
    if value != 0 and (abs(value) < 1e-3 or abs(value) >= 1e6):
        return f"{value:.{figures}e}".replace("e-0", "e&minus;").replace("e+0", "e")
    return f"{value:.{figures}f}"


def sig(value, figures=6):
    """A number for an SVG label, where width is scarce."""
    if value is None or isinstance(value, str):
        return "&mdash;"
    return f"{value:.{figures}g}"


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


# --------------------------------------------------------------------------
# Axis helpers
# --------------------------------------------------------------------------

def nice_steps(limit, count=5):
    """Round values from 0 up to `limit`, about `count` of them, none above it.

    The step is the round number NEAREST limit/count on a log scale, not the
    next one up. Rounding up costs a whole grid line whenever limit/count falls
    just above a round value, which on a pole map means an s-plane with two
    circles on it.
    """
    if limit <= 0:
        return []
    raw = limit / count
    magnitude = 10.0 ** math.floor(math.log10(raw))
    candidates = [f * magnitude for f in (1.0, 2.0, 2.5, 5.0, 10.0)]
    step = min(candidates, key=lambda c: abs(math.log(c / raw)))
    out, value = [], step
    while value <= limit * 1.0001 and len(out) < 12:
        out.append(round(value, 12))
        value += step
    return out


def decade_ticks(low, high):
    """Every power of ten inside [low, high], inclusive of the endpoints."""
    out = []
    exponent = math.ceil(math.log10(low) - 1e-9)
    while 10.0 ** exponent <= high * (1 + 1e-9):
        out.append(10.0 ** exponent)
        exponent += 1
    return out


def linear_ticks(low, high, count=6):
    span = high - low
    raw = span / count
    magnitude = 10.0 ** math.floor(math.log10(raw))
    for factor in (1.0, 2.0, 2.5, 5.0, 10.0):
        if raw <= factor * magnitude:
            step = factor * magnitude
            break
    out, value = [], math.ceil(low / step) * step
    while value <= high * (1 + 1e-9) + 1e-12:
        out.append(round(value, 12))
        value += step
    return out


# --------------------------------------------------------------------------
# Figures. Everything is inline SVG with CSS classes, so the plots re-theme
# with the page: a colour named here would be a colour the light variant and
# the print stylesheet could not reach.
# --------------------------------------------------------------------------

class Svg:
    def __init__(self, width, height, label):
        self.w, self.h = width, height
        self.parts = [
            f'<svg viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
            f'role="img" xmlns="http://www.w3.org/2000/svg" aria-label="{esc(label)}">'
        ]

    def add(self, markup):
        self.parts.append(markup)

    def text(self, x, y, body, cls="lbl", anchor="start", size=None, extra=""):
        style = f' font-size="{size}"' if size else ""
        self.add(f'<text class="{cls}" x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}"'
                 f'{style}{extra}>{body}</text>')

    def line(self, x1, y1, x2, y2, cls, extra=""):
        self.add(f'<line class="{cls}" x1="{x1:.1f}" y1="{y1:.1f}" '
                 f'x2="{x2:.1f}" y2="{y2:.1f}"{extra}/>')

    def path(self, points, cls, extra=""):
        if not points:
            return
        d = "M " + " L ".join(f"{x:.2f},{y:.2f}" for x, y in points)
        self.add(f'<path class="{cls}" d="{d}" fill="none"{extra}/>')

    def cross(self, x, y, cls, size=7.0):
        self.add(f'<g class="{cls}">'
                 f'<line x1="{x-size:.1f}" y1="{y-size:.1f}" x2="{x+size:.1f}" y2="{y+size:.1f}"/>'
                 f'<line x1="{x-size:.1f}" y1="{y+size:.1f}" x2="{x+size:.1f}" y2="{y-size:.1f}"/>'
                 f"</g>")

    def done(self):
        self.parts.append("</svg>")
        return "\n".join(self.parts)


class SPlane:
    """One s-plane panel, with EQUAL SCALE ON BOTH AXES.

    Not a stylistic choice. Constant-omega_n loci are circles about the origin
    and constant-zeta loci are rays through it; under unequal scaling the
    circles become ellipses and the rays stop reading as constant damping, so
    the grid would be decorative rather than something to measure against.
    """

    def __init__(self, x0, y0, size, re_lo, re_hi, clip_id):
        self.x0, self.y0, self.size, self.clip = x0, y0, size, clip_id
        self.x1, self.y1 = x0 + size, y0 + size
        self.re_lo, self.re_hi = re_lo, re_hi
        self.k = size / (re_hi - re_lo)
        self.im_max = (re_hi - re_lo) / 2.0
        self.ox = x0 + (0.0 - re_lo) * self.k
        self.oy = y0 + size / 2.0

    def X(self, re):
        return self.ox + re * self.k

    def Y(self, im):
        return self.oy - im * self.k

    def holds(self, mode):
        return (self.re_lo <= mode["re"] <= self.re_hi
                and abs(mode["im"]) <= self.im_max)


def splane_window(modes, pad_x=1.25, pad_y=2.3):
    """A window that holds every pole, with the axes still equally scaled.

    Because the scale is equal, the imaginary half-height is fixed by the real
    span: half of it. So the span has to satisfy BOTH the most negative real
    part and the largest imaginary part, and the wider requirement wins.
    """
    re_low = min(m["re"] for m in modes)
    im_high = max(abs(m["im"]) for m in modes)
    span = max(pad_x * abs(re_low), pad_y * im_high, 1e-9)
    re_hi = 0.05 * span
    return re_hi - span, re_hi


def pole_map(run):
    """The s-plane, whole and magnified, with the classifier's own labels."""
    modes = []
    for axis in run["axes"]:
        for mode in axis["modes"]:
            modes.append(dict(mode, axis=axis["axis"]))

    width, height, size = 980, 524, 380
    svg = Svg(width, height,
              "Pole map of the NT-33A at flight condition 1: every classical mode "
              "plotted on the s-plane, labelled by eigenvector participation.")

    lo, hi = splane_window(modes)
    whole = SPlane(58, 66, size, lo, hi, "clip-whole")

    # The magnified panel exists for the modes the whole map cannot resolve.
    # Which ones those are is decided from the data, not named here: anything
    # inside a fifth of the fastest mode's magnitude.
    fastest = max(abs(complex(m["re"], m["im"])) for m in modes)
    slow = [m for m in modes if abs(complex(m["re"], m["im"])) < 0.2 * fastest]
    detail = None
    if slow:
        d_lo, d_hi = splane_window(slow)
        detail = SPlane(width - size - 58, 66, size, d_lo, d_hi, "clip-detail")

    svg.add(f'<clipPath id="clip-whole"><rect x="{whole.x0}" y="{whole.y0}" '
            f'width="{size}" height="{size}"/></clipPath>')
    if detail:
        svg.add(f'<clipPath id="clip-detail"><rect x="{detail.x0}" y="{detail.y0}" '
                f'width="{size}" height="{size}"/></clipPath>')

    def draw(panel, caption):
        svg.add(f'<rect class="panel" x="{panel.x0}" y="{panel.y0}" '
                f'width="{panel.size}" height="{panel.size}" rx="3"/>')
        svg.add(f'<g clip-path="url(#{panel.clip})">')
        # The right half-plane, shaded: a pole there is a divergence.
        svg.add(f'<rect class="rhp" x="{panel.X(0):.1f}" y="{panel.y0}" '
                f'width="{max(0.0, panel.x1 - panel.X(0)):.1f}" height="{panel.size}"/>')

        # Constant omega_n: half-circles about the origin, drawn as polylines
        # so there is no arc sweep flag to get backwards.
        for radius in nice_steps(panel.im_max * 1.9):
            r = radius * panel.k
            pts = [(panel.ox + r * math.cos(t), panel.oy - r * math.sin(t))
                   for t in [math.pi / 2 + i * math.pi / 72 for i in range(73)]]
            svg.path(pts, "grid")
            svg.text(panel.X(-radius) + 4, panel.oy - 5, sig(radius, 3),
                     "tick", size=10)

        # Constant zeta: rays from the origin.
        zetas = [0.1, 0.2, 0.4, 0.6, 0.8]
        reach = panel.size * 1.6 / panel.k
        for index, zeta in enumerate(zetas):
            dx, dy = -zeta, math.sqrt(1.0 - zeta * zeta)
            for sign in (1, -1):
                svg.line(panel.ox, panel.oy,
                         panel.X(dx * reach), panel.Y(sign * dy * reach), "gridfaint")
            # Labels are staggered in depth. In the magnified panel every ray
            # is nearly vertical, and labelling them all at one height piles
            # five labels into thirty pixels of width.
            depth = (panel.y1 - panel.oy - 18) * (0.5 + 0.5 * index / (len(zetas) - 1))
            t = depth / (dy * panel.k)
            lx = panel.X(dx * t)
            if panel.x0 + 16 < lx < panel.x1 - 16:
                svg.text(lx, panel.oy + depth + 12, f"&zeta; {zeta:g}", "tick",
                         anchor="middle", size=10)

        svg.line(panel.x0, panel.oy, panel.x1, panel.oy, "axis")
        svg.line(panel.X(0), panel.y0, panel.X(0), panel.y1, "axis-crit")

        for mode in modes:
            if not panel.holds(mode):
                continue
            cls = "m-long" if mode["axis"] == "longitudinal" else "m-lat"
            svg.cross(panel.X(mode["re"]), panel.Y(mode["im"]), cls)
            if mode["oscillatory"]:
                svg.cross(panel.X(mode["re"]), panel.Y(-mode["im"]), cls)
        svg.add("</g>")

        svg.text(panel.x0 + 10, panel.y0 + 18, caption, "eyebrow", size=10)
        svg.text(panel.x0 + panel.size / 2, panel.y1 + 22,
                 "Re(&lambda;) &mdash; 1/s", "tick", anchor="middle", size=11)
        svg.text(panel.x0, panel.y0 - 12,
                 "Im(&lambda;) &mdash; rad/s", "tick", size=11)

    draw(whole, "EVERY MODE")
    if detail:
        draw(detail, f"MAGNIFIED &#183; {detail.k / whole.k:.0f}&#215;")
        # The magnified window, marked on the whole map and tied to it, so the
        # second panel is a place on the first rather than a second picture.
        bx0, bx1 = whole.X(detail.re_lo), whole.X(detail.re_hi)
        by0, by1 = whole.Y(detail.im_max), whole.Y(-detail.im_max)
        svg.add(f'<rect class="marker" x="{bx0:.1f}" y="{by0:.1f}" '
                f'width="{max(2.0, bx1 - bx0):.1f}" height="{max(2.0, by1 - by0):.1f}"/>')
        for from_y, to_y in ((by0, detail.y0), (by1, detail.y1)):
            svg.line(bx1, from_y, detail.x0, to_y, "marker-tie")

    # Labels for the poles, placed against the panel that resolves each one.
    for mode in modes:
        panel = detail if (detail and detail.holds(mode)) else whole
        if not panel.holds(mode):
            continue
        cls = "t-long" if mode["axis"] == "longitudinal" else "t-lat"
        x, y = panel.X(mode["re"]), panel.Y(mode["im"])
        if mode["oscillatory"]:
            svg.text(x - 13, y - 10, esc(mode["label"]), cls, anchor="end", size=13)
        else:
            svg.text(x, y - 16, esc(mode["label"]), cls, anchor="middle", size=13)

    svg.text(58, height - 14,
             "&#10005; a pole. An oscillatory mode is a conjugate pair and is drawn twice. "
             "Shaded: the right half-plane, where a pole is a divergence.",
             "note", size=11)
    return svg.done()


class LogPanel:
    """A panel with a logarithmic frequency axis and a linear value axis."""

    def __init__(self, x0, y0, width, height, w_lo, w_hi, v_lo, v_hi, clip_id):
        self.x0, self.y0, self.w, self.h, self.clip = x0, y0, width, height, clip_id
        self.x1, self.y1 = x0 + width, y0 + height
        self.lw_lo, self.lw_hi = math.log10(w_lo), math.log10(w_hi)
        self.v_lo, self.v_hi = v_lo, v_hi

    def X(self, frequency):
        return self.x0 + (math.log10(frequency) - self.lw_lo) / (self.lw_hi - self.lw_lo) * self.w

    def Y(self, value):
        return self.y1 - (value - self.v_lo) / (self.v_hi - self.v_lo) * self.h


def bode(run):
    """Magnitude and phase of the open loop, with every crossover marked."""
    loop = run["loop"]
    frequencies = loop["frequencies_rad_s"]
    margins = loop["margins"]

    width, height = 980, 604
    left, plot_w = 74, 848
    svg = Svg(width, height,
              "Bode plot of the open-loop bank-angle transfer function, with every "
              "gain crossover marked and the governing phase margin annotated.")

    w_lo, w_hi = frequencies[0], frequencies[-1]
    db = loop["magnitude_db"]
    deg = loop["phase_deg"]

    db_lo = math.floor(min(db) / 20.0) * 20.0
    db_hi = math.ceil(max(db) / 20.0) * 20.0
    # -180 is always in view even when the phase never reaches it: the fact
    # that it does not is the whole story of this loop's gain margin.
    deg_lo = min(-190.0, math.floor(min(deg) / 45.0) * 45.0)
    deg_hi = max(0.0, math.ceil(max(deg) / 45.0) * 45.0)

    magnitude = LogPanel(left, 44, plot_w, 200, w_lo, w_hi, db_lo, db_hi, "clip-mag")
    phase = LogPanel(left, 306, plot_w, 200, w_lo, w_hi, deg_lo, deg_hi, "clip-pha")
    for panel in (magnitude, phase):
        svg.add(f'<clipPath id="{panel.clip}"><rect x="{panel.x0}" y="{panel.y0}" '
                f'width="{panel.w}" height="{panel.h}"/></clipPath>')

    def frame(panel, ticks, unit, fmt="{:g}"):
        svg.add(f'<rect class="panel" x="{panel.x0}" y="{panel.y0}" '
                f'width="{panel.w}" height="{panel.h}" rx="3"/>')
        for value in ticks:
            y = panel.Y(value)
            svg.line(panel.x0, y, panel.x1, y, "grid")
            svg.text(panel.x0 - 8, y + 4, fmt.format(value), "tick", anchor="end", size=10)
        for decade in decade_ticks(w_lo, w_hi):
            x = panel.X(decade)
            svg.line(x, panel.y0, x, panel.y1, "grid")


    # The unit belongs to the panel heading, not to a second label beside the
    # axis: two of them collide at this width and say the same thing twice.
    frame(magnitude, linear_ticks(db_lo, db_hi, 5), "dB")
    frame(phase, linear_ticks(deg_lo, deg_hi, 5), "deg")

    # Reference lines: unity gain, and half a turn of phase lag.
    svg.line(magnitude.x0, magnitude.Y(0.0), magnitude.x1, magnitude.Y(0.0), "reference")
    svg.text(magnitude.x1 - 6, magnitude.Y(0.0) - 6, "|L| = 1", "t-crit",
             anchor="end", size=11)
    if deg_lo <= -180.0 <= deg_hi:
        svg.line(phase.x0, phase.Y(-180.0), phase.x1, phase.Y(-180.0), "reference")
        svg.text(phase.x1 - 6, phase.Y(-180.0) - 6, "&minus;180&deg;", "t-crit",
                 anchor="end", size=11)

    svg.add(f'<g clip-path="url(#clip-mag)">')
    svg.path([(magnitude.X(w), magnitude.Y(v)) for w, v in zip(frequencies, db)], "trace-mag")
    svg.add("</g>")
    svg.add(f'<g clip-path="url(#clip-pha)">')
    svg.path([(phase.X(w), phase.Y(v)) for w, v in zip(frequencies, deg)], "trace-pha")
    svg.add("</g>")

    # Every gain crossover, not just the governing one. A loop whose magnitude
    # crosses unity three times has three phase margins, and a picture showing
    # one would be hiding the other two.
    governing = margins.get("phase_margin_frequency_rad_s")
    for crossing in margins["gain_crossings"]:
        w = crossing["frequency_rad_s"]
        is_governing = governing is not None and abs(w - governing) < 1e-9
        cls = "crossing-strong" if is_governing else "crossing"
        svg.line(magnitude.X(w), magnitude.y0, magnitude.X(w), magnitude.y1, cls)
        svg.line(phase.X(w), phase.y0, phase.X(w), phase.y1, cls)
        svg.cross(magnitude.X(w), magnitude.Y(0.0), "m-crit", 5.0)
        if is_governing and deg_lo <= -180.0:
            # The phase margin, drawn as what it is: the gap between the phase
            # at this crossover and half a turn of lag.
            y_here = phase.Y(crossing["phase_margin_deg"] - 180.0)
            y_crit = phase.Y(-180.0)
            svg.line(phase.X(w), y_here, phase.X(w), y_crit, "span")
            svg.text(phase.X(w) + 8, (y_here + y_crit) / 2 + 4,
                     f'PM {crossing["phase_margin_deg"]:.1f}&deg;', "t-crit", size=12)
        svg.text(magnitude.X(w), magnitude.y0 - 6, f"{w:.3g}", "tick",
                 anchor="middle", size=10)

    for decade in decade_ticks(w_lo, w_hi):
        svg.text(phase.X(decade), phase.y1 + 19, f"{decade:g}", "tick",
                 anchor="middle", size=10)
    svg.text(left + plot_w / 2, phase.y1 + 44,
             "&omega; &mdash; rad/s", "tick", anchor="middle", size=12)

    svg.text(left, 28, "OPEN-LOOP MAGNITUDE &#183; dB", "eyebrow", size=10)
    svg.text(left, 290, "OPEN-LOOP PHASE &#183; deg, unwrapped", "eyebrow", size=10)
    svg.text(left, height - 12,
             "Vertical marks: every &#124;L(j&omega;)&#124; = 1 crossing. The bold one governs; "
             "the numbers above the upper panel are their frequencies in rad/s.",
             "note", size=11)
    return svg.done()


def nyquist(run):
    """The Nyquist curve against the disk the margin says it must avoid."""
    loop = run["loop"]
    disk = loop["disk"]
    re, im = loop["nyquist_re"], loop["nyquist_im"]

    g_min, g_max = disk["gain_variation_min"], disk["gain_variation_max"]
    have_disk = finite(g_min) and finite(g_max) and g_min > 0.0 and g_max > 0.0
    if have_disk:
        # The perturbed loop is f&#183;L for f in the disk, so the closed loop is
        # stable for all of them exactly when L avoids {-1/f}. That image is
        # itself a disk, and it meets the real axis at -1/gamma_min and
        # -1/gamma_max — the two gain limits the margin guarantees.
        left_x, right_x = -1.0 / g_min, -1.0 / g_max
        centre = (left_x + right_x) / 2.0
        radius = abs(left_x - right_x) / 2.0
    else:
        centre, radius = -1.0, 0.0

    size = 470
    svg = Svg(size + 340, size + 96,
              "Nyquist plot of the open loop with the disk-margin exclusion disk drawn: "
              "the curve is tangent to the disk at the critical frequency.")

    x_lo = min(-1.0, centre - radius) - 0.4
    x_hi = 1.35
    span = x_hi - x_lo
    k = size / span
    ox = 40 + (0.0 - x_lo) * k
    oy = 40 + size / 2.0

    def X(v):
        return ox + v * k

    def Y(v):
        return oy - v * k

    svg.add(f'<clipPath id="clip-nyq"><rect x="40" y="40" width="{size}" height="{size}"/>'
            f"</clipPath>")
    svg.add(f'<rect class="panel" x="40" y="40" width="{size}" height="{size}" rx="3"/>')
    svg.add('<g clip-path="url(#clip-nyq)">')

    for value in linear_ticks(x_lo, x_hi, 6):
        svg.line(X(value), 40, X(value), 40 + size, "grid")
    for value in linear_ticks(-span / 2, span / 2, 6):
        svg.line(40, Y(value), 40 + size, Y(value), "grid")
    svg.line(40, oy, 40 + size, oy, "axis")
    svg.line(X(0), 40, X(0), 40 + size, "axis")

    # The unit circle: where |L| = 1, so where the phase margin is measured.
    circle = [(X(math.cos(t)), Y(math.sin(t)))
              for t in [i * math.pi / 90 for i in range(181)]]
    svg.path(circle, "unit-circle")

    if have_disk:
        svg.add(f'<circle class="disk" cx="{X(centre):.1f}" cy="{Y(0.0):.1f}" '
                f'r="{radius * k:.1f}"/>')

    # The conjugate branch, faint: the Nyquist contour is the whole of it, and
    # a half-picture invites an encirclement count that is off by two.
    svg.path([(X(a), Y(-b)) for a, b in zip(re, im)], "trace-mirror")
    svg.path([(X(a), Y(b)) for a, b in zip(re, im)], "trace-nyq")

    svg.cross(X(-1.0), Y(0.0), "m-crit", 7.0)

    if have_disk and finite(disk["destabilising_perturbation_re"]):
        f = complex(disk["destabilising_perturbation_re"], disk["destabilising_perturbation_im"])
        if abs(f) > 0.0:
            touch = -1.0 / f
            svg.add(f'<circle class="touch" cx="{X(touch.real):.1f}" '
                    f'cy="{Y(touch.imag):.1f}" r="5"/>')
    svg.add("</g>")

    for value in linear_ticks(x_lo, x_hi, 6):
        svg.text(X(value), 40 + size + 18, f"{value:g}", "tick", anchor="middle", size=10)
    svg.text(40, 30, "Im L(j&omega;)", "tick", size=11)
    svg.text(40 + size / 2, 40 + size + 40, "Re L(j&omega;)", "tick",
             anchor="middle", size=11)

    # Legend. Every plotted quantity gets an entry, and every entry its units.
    lx, ly = size + 72, 74
    entries = [
        ("trace-nyq", "L(j&omega;), &omega; &gt; 0 &mdash; dimensionless"),
        ("trace-mirror", "L(&minus;j&omega;), the conjugate branch"),
        ("unit-circle", "&#124;L&#124; = 1"),
        ("m-crit", "the critical point, &minus;1 + 0j"),
    ]
    if have_disk:
        entries.append(("disk", "the disk L must avoid"))
        entries.append(("touch", "tangency, at &omega; = "
                                 f"{sig(disk['critical_frequency_rad_s'], 4)} rad/s"))
    for cls, label in entries:
        if cls == "m-crit":
            svg.cross(lx + 9, ly - 4, cls, 5.0)
        elif cls == "touch":
            svg.add(f'<circle class="touch" cx="{lx + 9}" cy="{ly - 4}" r="4"/>')
        elif cls == "disk":
            svg.add(f'<circle class="disk" cx="{lx + 9}" cy="{ly - 4}" r="7"/>')
        else:
            svg.line(lx, ly - 4, lx + 18, ly - 4, cls)
        svg.text(lx + 26, ly, label, "note", size=11.5)
        ly += 24
    return svg.done()


# --------------------------------------------------------------------------
# The stylesheet.
#
# Tokens on :root are the Twitter Dim palette; :root:has(#light:checked) and
# @media print swap in the light one. Nothing below names a colour outside the
# two token blocks, so a plot cannot be legible in one variant and not the
# other — which is the failure this page has to avoid, because it is meant to
# be read on a screen, printed, and projected.
# --------------------------------------------------------------------------

CSS = """
:root{
  --ground:#15202B; --panel:#1E2732; --ink:#F7F9F9; --ink-muted:#8B98A5;
  --rule:#38444D; --grid:#38444D; --accent:#1D9BF0; --link:#1D9BF0; --lateral:#FFD400;
  --phase:#9B7BFF; --critical:#F91880; --good:#00BA7C; --bad:#F87171;
  --rhp:rgba(249,24,128,.10); --code:#111A24;
}
:root:has(#light:checked){
  --ground:#FFFFFF; --panel:#F7F9F9; --ink:#0F1419; --ink-muted:#536471;
  --rule:#CFD9DE; --grid:#AFBBC3; --accent:#1D9BF0; --link:#1578C2; --lateral:#C2410C;
  --phase:#6941E0; --critical:#E01673; --good:#007A55; --bad:#D91F2C;
  --rhp:rgba(224,22,115,.08); --code:#F0F3F5;
}
*{box-sizing:border-box}
html{-webkit-text-size-adjust:100%;background:var(--ground)}
body{margin:0;background:var(--ground);color:var(--ink);
  font:16px/1.62 -apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif}
main{max-width:1040px;margin:0 auto;padding:0 24px 96px}
a{color:var(--link)}
a:hover{text-decoration:none}
h1{font-size:32px;line-height:1.18;letter-spacing:-.5px;margin:.15em 0 .3em;font-weight:600}
h2{font-size:22px;margin:2.6em 0 .5em;padding-bottom:.32em;font-weight:600;
  border-bottom:1px solid var(--rule)}
h3{font-size:16.5px;margin:1.8em 0 .4em;font-weight:600}
p{color:var(--ink);margin:.7em 0}
.eyebrow,.kicker{font-size:11.5px;letter-spacing:1.3px;text-transform:uppercase;
  color:var(--ink-muted);font-weight:600}
.sub{color:var(--ink-muted);font-size:17px;margin:.2em 0 1.1em}
.lede{font-size:17.5px;line-height:1.6}
header{border-bottom:1px solid var(--rule);background:var(--panel);margin-bottom:34px}
.bar{max-width:1040px;margin:0 auto;padding:26px 24px 30px;position:relative}
.meta{display:flex;flex-wrap:wrap;gap:6px 26px;font-size:13.5px;color:var(--ink-muted);
  margin-top:14px;padding-top:14px;border-top:1px solid var(--rule)}
.meta b{color:var(--ink);font-weight:600}
.cite{font-size:13.5px;color:var(--ink-muted);margin-top:10px;max-width:74ch}
.tw{overflow-x:auto;margin:1.1em 0}
table{border-collapse:collapse;width:100%;font-size:14px}
th,td{border:1px solid var(--rule);padding:7px 11px;text-align:left;vertical-align:top}
th{background:var(--panel);font-weight:600;white-space:nowrap;color:var(--ink)}
td.n,th.n{text-align:right;font-variant-numeric:tabular-nums;
  font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:13px}
tbody tr:nth-child(even) td{background:color-mix(in srgb,var(--panel) 55%,transparent)}
code{background:var(--code);border:1px solid var(--rule);border-radius:4px;padding:.1em .36em;
  font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:.86em}
pre{background:var(--code);border:1px solid var(--rule);border-radius:7px;padding:14px 16px;
  overflow-x:auto;font-size:13px;line-height:1.55}
pre code{background:none;border:0;padding:0}
figure{margin:1.4em 0 1.8em}
figure svg{width:100%;height:auto;display:block}
figcaption{font-size:13.5px;color:var(--ink-muted);margin-top:10px;max-width:86ch}
figcaption b{color:var(--ink);font-weight:600}
.note-box{background:var(--panel);border:1px solid var(--rule);border-left:3px solid var(--accent);
  border-radius:0 7px 7px 0;padding:13px 17px;margin:1.3em 0;font-size:14.5px;color:var(--ink-muted)}
.note-box.warn{border-left-color:var(--lateral)}
.note-box b{color:var(--ink)}
.routine{font-size:12.5px;color:var(--ink-muted);margin:.3em 0 0}
.routine code{font-size:12px}
footer{border-top:1px solid var(--rule);margin-top:60px;padding:22px 24px 60px;
  max-width:1040px;margin-left:auto;margin-right:auto;font-size:13px;color:var(--ink-muted)}
#light{position:absolute;opacity:0;pointer-events:none}
.switch{position:absolute;top:26px;right:24px;cursor:pointer;font-size:12.5px;
  color:var(--ink-muted);border:1px solid var(--rule);border-radius:999px;
  padding:5px 13px;user-select:none;background:var(--ground)}
.switch:hover{color:var(--ink)}
.deviation-in{color:var(--good);font-weight:600}
.deviation-out{color:var(--lateral);font-weight:600}

/* --- figure elements ---------------------------------------------------- */
svg text{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif}
.panel{fill:var(--panel);stroke:var(--rule);stroke-width:1}
.rhp{fill:var(--rhp)}
.grid{stroke:var(--grid);stroke-width:1;fill:none}
.gridfaint{stroke:var(--grid);stroke-width:1;stroke-dasharray:2 3}
.axis{stroke:var(--ink-muted);stroke-width:1.2;opacity:.75}
.axis-crit{stroke:var(--critical);stroke-width:1.2;opacity:.7}
.reference{stroke:var(--critical);stroke-width:1.2;stroke-dasharray:5 4}
.tick{fill:var(--ink-muted);font-size:10px}
.note{fill:var(--ink-muted);font-size:11px}
.eyebrow{fill:var(--ink-muted);letter-spacing:1.2px;font-weight:600}
.m-long,.m-long line{stroke:var(--accent);stroke-width:2.4;stroke-linecap:round}
.m-lat,.m-lat line{stroke:var(--lateral);stroke-width:2.4;stroke-linecap:round}
.m-crit,.m-crit line{stroke:var(--critical);stroke-width:2.2;stroke-linecap:round}
.t-long{fill:var(--accent);font-weight:600}
.t-lat{fill:var(--lateral);font-weight:600}
.t-crit{fill:var(--critical);font-weight:600}
.marker{fill:none;stroke:var(--ink-muted);stroke-width:1;stroke-dasharray:3 2}
.marker-tie{stroke:var(--ink-muted);stroke-width:1;stroke-dasharray:3 2;opacity:.4}
.trace-mag{stroke:var(--accent);stroke-width:2}
.trace-pha{stroke:var(--phase);stroke-width:2}
.trace-nyq{stroke:var(--accent);stroke-width:2}
.trace-mirror{stroke:var(--accent);stroke-width:1.2;stroke-dasharray:4 4;opacity:.5}
.unit-circle{stroke:var(--ink-muted);stroke-width:1.2;stroke-dasharray:3 3;opacity:.8}
.disk{fill:color-mix(in srgb,var(--critical) 10%,transparent);stroke:var(--critical);
  stroke-width:1.4}
.touch{fill:var(--critical);stroke:none}
.crossing{stroke:var(--ink-muted);stroke-width:1;stroke-dasharray:3 3;opacity:.6}
.crossing-strong{stroke:var(--critical);stroke-width:1.4;stroke-dasharray:4 3}
.span{stroke:var(--critical);stroke-width:3}

@media (max-width:760px){
  main{padding:0 14px 64px} .bar{padding:22px 14px 26px} h1{font-size:25px}
  .switch{position:static;display:inline-block;margin-top:12px}
}

/* Printed and projected, this page is read on white. A report printed from a
   dark palette is a report nobody reads twice. */
@media print{
  :root{
    --ground:#FFFFFF; --panel:#F7F9F9; --ink:#0F1419; --ink-muted:#536471;
    --rule:#CFD9DE; --grid:#AFBBC3; --accent:#1578C2; --link:#1578C2; --lateral:#C2410C;
    --phase:#6941E0; --critical:#E01673; --good:#007A55; --bad:#D91F2C;
    --rhp:rgba(224,22,115,.08); --code:#F0F3F5;
  }
  .switch,#light{display:none}
  header{border-bottom:1px solid var(--rule)}
  figure,table{break-inside:avoid;page-break-inside:avoid}
  h2{break-after:avoid;page-break-after:avoid}
  a{text-decoration:none}
  a[href^="http"]::after{content:" (" attr(href) ")";font-size:10px;color:#536471}
  figcaption a[href^="http"]::after{content:""}
}
"""


# --------------------------------------------------------------------------
# The page
# --------------------------------------------------------------------------

def table(headers, rows, numeric=()):
    """A table. `numeric` names the column indices that are right-aligned."""
    head = "".join(f'<th class="{"n" if i in numeric else ""}">{h}</th>'
                   for i, h in enumerate(headers))
    body = []
    for row in rows:
        cells = "".join(f'<td class="{"n" if i in numeric else ""}">{c}</td>'
                        for i, c in enumerate(row))
        body.append(f"<tr>{cells}</tr>")
    return ('<div class="tw"><table><thead><tr>' + head + "</tr></thead><tbody>"
            + "".join(body) + "</tbody></table></div>")


def routine(text):
    return f'<p class="routine">Computed by <code>{esc(text)}</code>.</p>'


def all_modes(run):
    out = []
    for axis in run["axes"]:
        for mode in axis["modes"]:
            out.append(dict(mode, axis=axis["axis"]))
    return out


def section_trim(run):
    trim = run["trim"]
    rows = [
        ["Altitude", num(trim["altitude_m"], 1), "m, geometric"],
        ["True airspeed", num(trim["airspeed_m_s"], 4), "m/s"],
        ["Mach", num(trim["mach"], 4), "&mdash;"],
        ["Dynamic pressure", num(trim["dynamic_pressure_pa"], 1), "Pa"],
        ["Angle of attack", num(trim["alpha_deg"], 4), "deg"],
        ["Flight-path angle", num(trim["flight_path_angle_deg"], 4), "deg"],
        ["Pitch attitude", num(trim["pitch_attitude_deg"], 4), "deg"],
        ["Elevator", num(trim["elevator_deg"], 4), "deg"],
        ["Thrust", num(trim["thrust_n"], 1), "N"],
        ["Trim lift coefficient", num(trim["lift_coefficient"], 5), "&mdash;"],
    ]
    envelope = ""
    if trim["outside_advisory_envelope"]:
        envelope = ('<p class="note-box warn"><b>Outside the model&rsquo;s advisory '
                    "envelope.</b> This derivative set was built about one reference "
                    "condition and returns a confident answer well away from it.</p>")
    return f"""
<h2 id="trim">The trim point</h2>
<p>Three unknowns &mdash; angle of attack, elevator and thrust &mdash; and three
equations: the two body-axis translational accelerations and the pitching
acceleration all vanish. Everything further down this page is taken about
<em>this</em> point, so it is the first thing to disbelieve.</p>
{table(["Quantity", "Value", "Unit"], rows, numeric={1})}
<p><b>Evidence.</b> Residual norm {num(run['trim']['residual_norm'], 12)}
(m/s&sup2; and rad/s&sup2;); trim Jacobian condition number
{num(run['trim']['jacobian_condition_number'], 0)}.</p>
<p class="note-box">A trim is only as good as its residual, so the residual is
reported rather than asserted, and the solver <b>throws</b> rather than
returning a best effort: a linearisation taken about a point that is not an
equilibrium produces a state-space model that is plausible and wrong. The
condition number is large here for a units reason and not a physical one &mdash;
an angle of order 0.04&nbsp;rad and a thrust of order 10<sup>4</sup>&nbsp;N sit
in the same unknown vector, so the columns differ in scale by five orders before
any aircraft is involved. It is worth worrying about only when it is large
<em>and</em> the residual will not come down.</p>
{envelope}
{routine(run['trim']['routine'])}
"""


def section_modes(run):
    rows = []
    for mode in all_modes(run):
        eigenvalue = num(mode["re"], 5)
        if mode["oscillatory"]:
            eigenvalue += f' &plusmn; {num(mode["im"], 5)}j'
        rows.append([
            f'<b>{esc(mode["label"])}</b>',
            esc(mode["axis"]),
            eigenvalue,
            num(mode["omega_n"], 5),
            num(mode["zeta"], 5) if mode["oscillatory"] else "&mdash;",
            num(mode["period_s"], 3),
            num(mode["time_to_half_s"], 3),
            num(mode["time_to_double_s"], 3),
            num(mode["label_score"], 3),
            esc(mode["label_reason"]),
        ])
    headers = ["Mode", "Axis", "&lambda; (1/s)", "&omega;<sub>n</sub> (rad/s)",
               "&zeta;", "Period (s)", "T&frac12; (s)", "T&times;2 (s)",
               "Score", "Why it carries that label"]
    return f"""
<h2 id="modes">The modal table</h2>
<p>The labels are not assigned by frequency order. Each one comes from
<em>eigenvector participation</em> &mdash; how much of the mode lives in the
states that define it &mdash; and the score is the share of participation that
does. Below about 0.5 a label is a guess, and the column is there so that can be
seen rather than assumed.</p>
{table(headers, rows, numeric={2, 3, 4, 5, 6, 7, 8})}
<p class="note-box">A dash is a quantity that is not defined for that mode, never
a zero: a real root has no period, and a mode has either a time to half amplitude
or a time to double, never both. Printing zero there would invite a plot to draw
it.</p>
{routine(run['axes'][0]['modes_routine'])}
"""


def section_participation(run):
    blocks = []
    for axis in run["axes"]:
        if not axis["participation_is_meaningful"]:
            blocks.append(
                f'<h3>{esc(axis["axis"]).capitalize()}</h3><p class="note-box warn">'
                "<b>Participation factors are not reported.</b> The eigenvector matrix "
                f'has a condition number of {num(axis["eigenvector_condition_number"], 2)}, '
                "so the eigenvectors are nearly linearly dependent and participation "
                "would be numerical noise. The eigenvalues are still well defined.</p>")
            continue
        headers = ["Mode"] + [f"<code>{esc(s)}</code>" for s in axis["states"]]
        rows = [[f'<b>{esc(m["label"])}</b>'] + [num(p, 3) for p in m["participation"]]
                for m in axis["modes"]]
        blocks.append(
            f'<h3>{esc(axis["axis"]).capitalize()} &mdash; states '
            f'<code>{esc(axis["selection"])}</code></h3>'
            + table(headers, rows, numeric=set(range(1, len(headers))))
            + '<p class="routine">Eigenvector matrix condition number '
              f'{num(axis["eigenvector_condition_number"], 2)}.</p>')
    return f"""
<h2 id="participation">Participation factors &mdash; the evidence for the labels</h2>
<p>Normalised to sum to one across the states of each axis. This is the measure
the classification rests on, so a label whose evidence looks thin here is a label
to distrust.</p>
{''.join(blocks)}
"""


def section_published(run):
    # The worst deviation is FOUND, not named. A sentence saying which quantity is
    # the worst is a claim about the run, and typing it here would make it a claim
    # that stops being true the first time the model moves — which is the same
    # mistake tools/validation/report_main.cpp refuses to let a case note make.
    worst = max(run["published"], key=lambda e: e["deviation_percent"])
    rows = []
    for entry in run["published"]:
        inside = abs(entry["computed"] - entry["published"]) <= entry["printed_precision"]
        verdict = ('<span class="deviation-in">inside</span>' if inside
                   else '<span class="deviation-out">outside</span>')
        rows.append([
            f'<b>{esc(entry["mode"])}</b>',
            esc(entry["quantity"]),
            num(entry["published"], 5),
            f'&plusmn;{sig(entry["printed_precision"], 2)}',
            num(entry["computed"], 6),
            f'{entry["deviation_percent"]:.2f}%',
            verdict,
        ])
    return f"""
<h2 id="published">Against the published values</h2>
<p>The input to everything above is a set of <b>non-dimensional</b> derivatives
and some geometry. There is no matrix anywhere in it. These are the modal
characteristics the same report printed, read from
<a href="{BLOB}tests/validation/reference/nt33a_fc1.csv">the committed reference
file</a> by the same loader the validation tier uses.</p>
{table(["Mode", "Quantity", "Published", "Its own printing", "galata",
        "Deviation", "Within the published printing?"],
       rows, numeric={2, 3, 4, 5})}
<p class="note-box"><b>&ldquo;Its own printing&rdquo; is not the gate.</b> The
source prints three significant figures, so each published value carries half a
unit in its last digit &mdash; that is the column above. The full error budget
also has to carry the rounding of the ten printed <em>inputs</em> the value was
computed from, and it is derived per quantity in
<a href="{BLOB}tests/validation/test_nt33a_modes.cpp">the validation tests</a>
and published in <a href="{SITE}verification.html">the V&amp;V report</a>. This
page reports the measurement; it does not carry a second copy of the gate,
because two gates are two answers to one question.</p>
<p class="note-box warn"><b>The largest deviation in this table is the
{esc(worst["mode"])} {esc(worst["quantity"])}, at {worst["deviation_percent"]:.2f}%.</b>
It is reported rather than absorbed by a wider tolerance, which is the whole
discipline: a deviation over budget gets localised and published, never widened
away. One quantity in this reference case does <em>not</em> reproduce at all
&mdash; the phugoid damping ratio of a state matrix assembled <em>by hand</em>
from the report&rsquo;s <em>dimensional</em> derivatives, where the chain above
reproduces it to the deviation shown in the first row. That gap is localised to a
single matrix entry and written up at
<a href="{SITE}notes/phugoid-damping.html">the phugoid-damping note</a>; its size
is published in <a href="{SITE}verification.html">the V&amp;V report</a>, which
measures it rather than quoting it. It is still open, and it is still listed.</p>
"""


def section_loop(run):
    loop = run["loop"]
    margins = loop["margins"]
    disk = loop["disk"]
    sens = loop["sensitivity"]

    crossings = [[
        "|L| = 1",
        num(c["frequency_rad_s"], 5),
        f'{num(c["phase_margin_deg"], 3)} deg',
        f'{num(c["delay_margin_s"], 5)} s',
    ] for c in margins["gain_crossings"]]
    for c in margins["phase_crossings"]:
        crossings.append([
            "phase = &minus;180&deg;",
            num(c["frequency_rad_s"], 5),
            f'{num(c["gain_margin_db"], 3)} dB',
            "&mdash;",
        ])

    governing = [
        ["Gain margin",
         "infinite" if not margins["has_gain_margin"]
         else f'{num(margins["gain_margin"], 4)} ({num(margins["gain_margin_db"], 2)} dB)',
         num(margins["gain_margin_frequency_rad_s"], 5)],
        ["Phase margin",
         "infinite" if not margins["has_phase_margin"]
         else f'{num(margins["phase_margin_deg"], 3)} deg',
         num(margins["phase_margin_frequency_rad_s"], 5)],
        ["Delay margin",
         "none" if not margins["has_delay_margin"]
         else f'{num(margins["delay_margin_s"], 5)} s',
         num(margins["delay_margin_frequency_rad_s"], 5)],
    ]

    gain_range = ("unbounded above" if not disk["gain_variation_is_bounded"]
                  else f'{num(disk["gain_variation_min"], 4)} to '
                       f'{num(disk["gain_variation_max"], 4)} '
                       f'({num(disk["gain_variation_min_db"], 2)} to '
                       f'{num(disk["gain_variation_max_db"], 2)} dB)')
    phase_range = ("any phase" if not disk["phase_variation_is_bounded"]
                   else f'&plusmn;{num(disk["phase_variation_deg"], 3)} deg')
    disk_rows = [
        ["Disk margin &alpha;", num(disk["alpha"], 5), "&mdash;"],
        ["Peak of |S + (&sigma;&minus;1)/2|", num(disk["peak_gain"], 5), "&mdash;"],
        ["Critical frequency", num(disk["critical_frequency_rad_s"], 5), "rad/s"],
        ["Guaranteed gain range", gain_range, "&mdash;"],
        ["Guaranteed phase range", phase_range, "&mdash;"],
        ["Sensitivity peak M<sub>S</sub>", num(sens["m_s"], 5),
         f'at {num(sens["m_s_frequency_rad_s"], 4)} rad/s'],
        ["Complementary peak M<sub>T</sub>", num(sens["m_t"], 5),
         f'at {num(sens["m_t_frequency_rad_s"], 4)} rad/s'],
    ]

    guaranteed = ""
    if sens["guaranteed_applies"] and sens["guaranteed_valid"]:
        guaranteed = f"""
<h3>What those peaks <em>guarantee</em></h3>
<p>Skogestad &amp; Postlethwaite, 2nd&nbsp;ed., equations (2.47) and (2.48).
Lower bounds: the loop&rsquo;s actual margins are at least this good. The two
gain-margin bounds have different functional forms &mdash;
M<sub>S</sub>/(M<sub>S</sub>&minus;1) against 1&nbsp;+&nbsp;1/M<sub>T</sub>
&mdash; which is easy to blur from memory.</p>
{table(["From", "Gain margin at least", "Phase margin at least"],
       [["M<sub>S</sub>", num(sens["gain_margin_from_m_s"], 4),
         f'{num(sens["phase_margin_from_m_s_deg"], 3)} deg'],
        ["M<sub>T</sub>", num(sens["gain_margin_from_m_t"], 4),
         f'{num(sens["phase_margin_from_m_t_deg"], 3)} deg']],
       numeric={1, 2})}
"""

    return f"""
<h2 id="loop">One closed loop</h2>
<p class="note-box"><b>The loop is not the aircraft.</b> Margins are a property
of a <em>loop</em>, and an aircraft on its own is not one. What is measured below
is the smallest control law that makes the question meaningful: measure bank
angle, multiply by a gain, drive the aileron. The A and B matrices are the
lateral linearisation from the run above &mdash; not a matrix read from a file
&mdash; and the C row is the control law.</p>
{table(["Property", "Value"],
       [["Loop", f'<code>{esc(loop["input"])}</code> to '
                 f'<code>{esc(loop["output"])}</code>'],
        ["Feedback gain", f'{num(loop["gain_rad_per_rad"], 2)} rad aileron per rad '
                          "of bank error"],
        ["Where the gain comes from", esc(loop["gain_provenance"])],
        ["Where the loop is broken", esc(loop["broken_at"])],
        ["Band searched", f'{num(margins["searched_from_rad_s"], 5)} to '
                          f'{num(margins["searched_to_rad_s"], 1)} rad/s, '
                          f'{margins["grid_points"]} points'],
        ["Worst pivot ratio over the sweep", num(loop["pivot_ratio_min"], 5)]])}
<p class="note-box warn"><b>That gain is a choice, not a published value.</b> It
is {num(loop["gain_rad_per_rad"], 2)}, and it is chosen because at that gain the
magnitude crosses unity {len(margins["gain_crossings"])} times, so the loop has
that many phase margins &mdash; which is the point worth showing. Breaking the loop at the plant <em>output</em> instead would give
a different transfer function and different margins for the same closed-loop
system. galata will not make that decision for you.</p>

<figure>
{bode(run)}
<figcaption><b>Open-loop frequency response.</b> Magnitude in dB and unwrapped
phase in degrees, against frequency in rad/s. Vertical marks are every
|L(j&omega;)| = 1 crossing; the bold one governs. Evaluated by Hessenberg solves
&mdash; the inverse is never formed &mdash; on a grid refined around the
loop&rsquo;s own lightly damped modes. Computed by
<code>{esc(loop["freqresp_routine"])}</code>.</figcaption>
</figure>

<h3>The governing margins</h3>
{table(["Margin", "Value", "At (rad/s)"], governing, numeric={1, 2})}

<h3>Every crossover, not just the governing one</h3>
<p>A loop whose magnitude crosses unity three times has three phase margins. An
implementation that returned the first crossing it found would report this loop
as comfortable at the largest of them.</p>
{table(["Kind", "&omega; (rad/s)", "Margin", "Delay"], crossings, numeric={1, 2, 3})}
{routine(loop["margins_routine"])}
<p class="note-box">Crossovers are found by <b>searching a frequency grid</b>. A
crossover pair narrower than the grid spacing is not found, which is why the band
and the point count travel with the result. Nothing here proves closed-loop
stability either: margins are distances from the critical point, not a Nyquist
encirclement count.</p>

<h2 id="disk">Gain and phase together</h2>
<p>The gain margin is the tolerable gain change with <em>no</em> phase change;
the phase margin is the tolerable phase change with <em>no</em> gain change. No
real actuator varies one alone. The disk margin asks how much of both together,
and the picture below is the whole of the argument: the perturbed loop is
f&#183;L for every f in a disk, so the closed loop is stable for all of them
exactly when L(j&omega;) misses the shaded region.</p>

<figure>
{nyquist(run)}
<figcaption><b>Nyquist, against the disk the loop must avoid.</b> Axes are the
real and imaginary parts of L(j&omega;), dimensionless. The shaded disk meets the
real axis at &minus;1/&gamma;<sub>min</sub> and &minus;1/&gamma;<sub>max</sub>,
the two gain limits the margin guarantees, and the marked point is where the
curve touches it &mdash; the boundary perturbation
f = {num(disk["destabilising_perturbation_re"], 4)}
{"&minus;" if disk["destabilising_perturbation_im"] < 0 else "+"}
{num(abs(disk["destabilising_perturbation_im"]), 4)}j places a closed-loop pole
exactly at s = j{num(disk["critical_frequency_rad_s"], 4)}. The low-frequency arm
runs off the panel. Computed by
<code>{esc(loop["disk_routine"])}</code>.</figcaption>
</figure>

{table(["Quantity", "Value", "Where"], disk_rows, numeric={1})}
{guaranteed}
<p class="note-box"><b>Every peak on this page is a grid maximum.</b> The disk
margin, M<sub>S</sub> and M<sub>T</sub> are all found by searching a refined
frequency grid and not by the exact Hamiltonian-eigenvalue method, so each peak
is a lower bound on the true H-infinity norm and each derived margin is an
<em>upper</em> bound on the true one. The error is in the optimistic direction.
Treat a marginal result as marginal.</p>
"""


def section_matrices(run):
    blocks = []
    for axis in run["axes"]:
        names = axis["states"]
        lines = [" " * 8 + "".join(f"{n:>12}" for n in names)]
        for i, row in enumerate(axis["a"]):
            lines.append(f"{names[i]:>7} " + "".join(f"{v:>12.6f}" for v in row))
        matrix_text = "\n".join(lines)

        inputs = axis["inputs"]
        b_lines = [" " * 8 + "".join(f"{n:>12}" for n in inputs)]
        for i, row in enumerate(axis["b"]):
            b_lines.append(f"{names[i]:>7} " + "".join(f"{v:>12.6f}" for v in row))
        b_text = "\n".join(b_lines)

        blocks.append(f"""
<h3>{esc(axis["axis"]).capitalize()}</h3>
<p class="routine">Rows and columns in the order
<code>{esc(', '.join(names))}</code>. Velocities m/s, angles rad, rates rad/s.
Worst relative truncation {sig(axis["worst_relative_truncation"], 3)};
neglected coupling {sig(axis["neglected_coupling"], 3)};
Euler-chart conditioning |cos&nbsp;&theta;| = {num(axis["chart_conditioning"], 6)}.</p>
<pre><code>A =
{esc(matrix_text)}

B =
{esc(b_text)}</code></pre>""")

    return f"""
<h2 id="matrices">The state matrices these came from</h2>
<p>Central differences about the trim, with a Richardson truncation estimate per
entry. The <em>neglected coupling</em> is the largest entry of the full
12&times;12 Jacobian that ties a retained state to a discarded one, relative to
the largest retained entry: it is measured rather than assumed, because
&ldquo;the longitudinal and lateral axes decouple&rdquo; is true at a
wings-level symmetric trim and false in a turn, and the difference does not show
in the reduced matrix.</p>
{''.join(blocks)}
{routine(run['axes'][0]['routine'])}
"""


def build(run):
    model = run["model"]
    trim = run["trim"]
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NT-33A at flight condition 1 &mdash; galata</title>
<meta name="description" content="Trim, the five classical modes, the pole map and one
closed bank-angle loop for the NT-33A at NASA CR-2144 flight condition 1, drawn from a
galata run rather than transcribed.">
<style>{CSS}</style>
</head>
<body>
<input type="checkbox" id="light">
<header><div class="bar">
<label class="switch" for="light">Light / dark</label>
<p class="kicker">galata &#183; flight-condition report</p>
<h1>NT-33A at flight condition 1</h1>
<p class="sub">Trim, the five classical modes, and one closed bank-angle loop
&mdash; every number on this page produced by the run that generated it.</p>
<div class="meta">
<span><b>Model</b> {esc(model["description"])}</span>
</div>
<div class="meta">
<span><b>Condition</b> {num(trim["altitude_m"], 0)} m, {num(trim["airspeed_m_s"], 3)} m/s,
M&nbsp;{num(trim["mach"], 4)}</span>
<span><b>Built by</b> galata {esc(run["version"])}</span>
<span><b>Emitter</b> <code>{esc(run["generator"])}</code></span>
<span><a href="{SITE}">Documentation</a></span>
<span><a href="{REPO}">Source</a></span>
</div>
<p class="cite"><b>Reference.</b> {esc(model["citation"])}</p>
</div></header>

<main>
<p class="lede">This page is a <em>run</em>, not a write-up. A single command
loads a nonlinear aircraft model built from non-dimensional derivatives, trims
it, linearises it about that trim by central differences, classifies the modes by
eigenvector participation, closes one loop around the result and measures it.
Every table cell and every mark on every plot below is read from the JSON that
run emitted; the routine that produced each is named beneath it.</p>

<p class="note-box"><b>How to get this page yourself.</b></p>
<pre><code>cmake --preset dev
cmake --build --preset dev
./build/dev/tools/report/galata-report-data &gt; docs/assets/nt33a-fc1-run.json
python3 scripts/gen-report-page.py</code></pre>
<p class="routine">CI regenerates the JSON and compares it numerically against
the committed copy, then regenerates this page and diffs it. A page that had
drifted from the code would still look like evidence, which is worse than no page
at all.</p>

{section_trim(run)}
{section_modes(run)}

<h2 id="poles">The pole map</h2>
<figure>
{pole_map(run)}
<figcaption><b>Every mode of both axes on one s-plane</b>, in units of 1/s
horizontally and rad/s vertically, with constant-&omega;<sub>n</sub> circles and
constant-&zeta; rays about the origin. Modes of the
<b style="color:var(--accent)">longitudinal</b> axis and the
<b style="color:var(--lateral)">lateral</b> axis carry different colours. The
axes are equally scaled, which is not a stylistic choice: under unequal scaling
the constant-&omega;<sub>n</sub> circles become ellipses and the
constant-&zeta; rays stop reading as constant damping. The labels are the
classifier&rsquo;s own output.</figcaption>
</figure>

{section_participation(run)}
{section_published(run)}
{section_loop(run)}
{section_matrices(run)}

<h2 id="limits">What this page is not</h2>
<p class="note-box warn"><b>Not certified, and not a handling-qualities
assessment.</b> Nothing in galata is DO-178C qualified and none of it may be used
as evidence in a certification package. Nothing here says whether a mode or a
margin is <em>acceptable</em>: Level 1/2/3 boundaries are a judgement against a
specification, and that is a separate capability with its own citations, which
galata does not have.</p>
<p class="note-box">The aircraft model is a first-order expansion about
<em>one</em> flight condition. It has no stall, no Mach effects and no engine,
and it will return a confident answer at any angle of attack it is asked for.
The linearisation is only as good as the trim, and the trim is reported above
with its residual so that can be checked. The classification cannot tell you that
a label is meaningless &mdash; only that its evidence was weak, which is what the
score column is for.</p>
</main>

<footer>
Generated by <code>scripts/gen-report-page.py</code> from
<code>docs/assets/nt33a-fc1-run.json</code>, emitted by
<code>tools/report/galata-report-data</code> at galata {esc(run["version"])}.
No hand-entered numbers, no network requests, no scripts.
&middot; <a href="{SITE}verification.html">V&amp;V report</a>
&middot; <a href="{REPO}">github.com/celikgo/galata</a>
&middot; Apache-2.0.
</footer>
</body>
</html>
"""


def main():
    check = "--check" in sys.argv[1:]
    if not DATA.exists():
        sys.exit(
            f"{DATA.relative_to(ROOT)} is missing.\n"
            "Build and run the emitter first, so this page is a run and not a write-up:\n"
            "  cmake --build --preset dev --target galata-report-data\n"
            "  ./build/dev/tools/report/galata-report-data > "
            f"{DATA.relative_to(ROOT)}")

    page = build(json.loads(DATA.read_text(encoding="utf-8")))

    if check:
        if not OUT.exists():
            sys.exit(f"::error::{OUT.relative_to(ROOT)} is missing; regenerate it with "
                     "python3 scripts/gen-report-page.py")
        if OUT.read_text(encoding="utf-8") != page:
            sys.exit(
                f"::error::{OUT.relative_to(ROOT)} is stale — it no longer matches what\n"
                f"{DATA.relative_to(ROOT)} and this script produce. Regenerate it with:\n"
                "  python3 scripts/gen-report-page.py")
        print(f"{OUT.relative_to(ROOT)} is current ({len(page)} bytes).")
        return

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(page, encoding="utf-8")
    print(f"wrote {OUT.relative_to(ROOT)} ({len(page)} bytes) from {DATA.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
