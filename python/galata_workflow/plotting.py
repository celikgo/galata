"""Dependency-light offline plots with explicit data contracts."""

from __future__ import annotations

import html
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Union

from .workflow import load_csv


PathLike = Union[Path, str]


def _scale(values: Sequence[float], lo: float, hi: float, start: float, size: float) -> List[float]:
    span = hi - lo if hi > lo else 1.0
    return [start + (value - lo) * size / span for value in values]


def _unit_for(name: str) -> str:
    units = {
        "_rad_s": "rad/s", "_rad": "rad", "_m_s": "m/s", "_m": "m",
        "_n_m": "N m", "_n": "N", "_kg": "kg", "_kg_m2": "kg m²",
    }
    for suffix, unit in units.items():
        if name.endswith(suffix):
            return unit
    return "1"


def _frame_for(name: str) -> str:
    if name.startswith(("position_", "velocity_", "wind_")):
        return "NED" if any(axis in name for axis in ("north", "east", "down")) else "body"
    if name.startswith(("roll", "pitch", "yaw", "quaternion_", "angular_rate_")):
        return "body"
    return "declared"


def _validate_rows(rows: Sequence[Mapping[str, Optional[float]]], channels: Sequence[str]) -> None:
    if not rows:
        raise ValueError("plot requires at least one row")
    if "time_s" not in rows[0]:
        raise ValueError("plot input must declare a time_s column")
    missing = [name for name in channels if name not in rows[0]]
    if missing:
        raise KeyError(f"missing named channels: {missing}")
    previous = None
    for index, row in enumerate(rows):
        time = row.get("time_s")
        if time is None or not math.isfinite(time):
            raise ValueError(f"time_s is missing or non-finite at row {index}")
        if previous is not None and time < previous:
            raise ValueError("time_s must be non-decreasing")
        previous = time


def _svg_frame(title: str, width: int = 960, height: int = 520) -> List[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">',
        "<style>text{font-family:system-ui,sans-serif;fill:#243447} "
        ".axis{stroke:#496273;stroke-width:1}.trace{fill:none;stroke-width:2}</style>",
        f'<text x="60" y="30" font-size="18">{html.escape(title)}</text>',
    ]


def _segments(rows: Sequence[Mapping[str, Optional[float]]], channel: str):
    segment: List[tuple[float, float]] = []
    for row in rows:
        time, value = row["time_s"], row[channel]
        if time is None or value is None:
            if segment:
                yield segment
                segment = []
        else:
            segment.append((time, value))
    if segment:
        yield segment


def _time_plot(rows: Sequence[Mapping[str, Optional[float]]], channels: Sequence[str], title: str,
               markers: Optional[Mapping[str, Sequence[float]]] = None) -> str:
    _validate_rows(rows, channels)
    width, height, left, right, top, bottom = 960, 520, 70, 30, 55, 65
    time = [float(row["time_s"]) for row in rows if row["time_s"] is not None]
    all_values = [value for name in channels for segment in _segments(rows, name)
                  for _, value in segment]
    if not all_values:
        raise ValueError("plot has no finite channel observations")
    output = _svg_frame(title, width, height)
    output.append(f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}"/>')
    output.append(f'<line class="axis" x1="{left}" y1="{height-bottom}" '
                  f'x2="{width-right}" y2="{height-bottom}"/>')
    output.append(f'<text x="{width/2:.0f}" y="{height-15}" text-anchor="middle">time (s)</text>')
    low, high = min(all_values), max(all_values)
    colours = ("#086788", "#d1495b", "#2a9d8f", "#f29e4c")
    for index, name in enumerate(channels):
        colour = colours[index % len(colours)]
        for segment in _segments(rows, name):
            x = _scale([point[0] for point in segment], min(time), max(time),
                       left, width - left - right)
            y = _scale([point[1] for point in segment], low, high,
                       height - bottom, -(height - bottom - top))
            points = " ".join(f"{xx:.2f},{yy:.2f}" for xx, yy in zip(x, y))
            output.append(f'<polyline class="trace" points="{points}" stroke="{colour}"/>')
        label = f"{name} [{_unit_for(name)}; {_frame_for(name)}]"
        output.append(f'<text x="{left+10+index*220}" y="{height-38}" fill="{colour}">'
                      f'{html.escape(label)}</text>')
    if markers:
        marker_colours = ("#6a4c93", "#1982c4", "#8ac926", "#ff595e")
        for marker_index, (label, times) in enumerate(markers.items()):
            colour = marker_colours[marker_index % len(marker_colours)]
            for marker_time in times:
                marker_x = _scale([marker_time], min(time), max(time),
                                  left, width - left - right)[0]
                output.append(f'<line x1="{marker_x:.2f}" y1="{top}" x2="{marker_x:.2f}" '
                              f'y2="{height-bottom}" stroke="{colour}" stroke-dasharray="4,3"/>')
            output.append(f'<text x="{left+10+marker_index*160}" y="{top-12}" fill="{colour}">'
                          f'{html.escape(label)}</text>')
    output.append(f'<text x="12" y="{top+10}">{high:.5g}</text>')
    output.append(f'<text x="12" y="{height-bottom}">{low:.5g}</text>')
    output.append("</svg>")
    return "\n".join(output)


def plot_time_history(csv_path: PathLike, channels: Sequence[str], output: PathLike,
                      title: str = "Galata time history",
                      markers: Optional[Mapping[str, Sequence[float]]] = None) -> Path:
    target = Path(output)
    target.write_text(_time_plot(load_csv(csv_path), channels, title, markers))
    return target


def plot_time_history_from_rows(rows: Sequence[Mapping[str, Optional[float]]], channels: Sequence[str],
                                output: PathLike, title: str,
                                markers: Optional[Mapping[str, Sequence[float]]] = None) -> Path:
    target = Path(output)
    target.write_text(_time_plot(rows, channels, title, markers))
    return target


def _channel(rows: Sequence[Mapping[str, Optional[float]]], signal: str) -> str:
    if not rows:
        raise ValueError("CSV has no data rows")
    if signal in rows[0]:
        return signal
    prefixed = f"output_{signal}"
    if prefixed in rows[0]:
        return prefixed
    raise KeyError(signal)


def _interpolate(rows: Sequence[Mapping[str, Optional[float]]], channel: str,
                 times: Sequence[float]) -> List[Optional[float]]:
    source = [(row["time_s"], row[channel]) for row in rows
              if row["time_s"] is not None and row[channel] is not None]
    if len(source) < 2:
        raise ValueError(f"cannot resample {channel}: fewer than two finite samples")
    result: List[Optional[float]] = []
    for time in times:
        if time < source[0][0] or time > source[-1][0]:
            result.append(None)
            continue
        for (left_t, left_v), (right_t, right_v) in zip(source, source[1:]):
            if left_t <= time <= right_t:
                fraction = 0.0 if right_t == left_t else (time-left_t)/(right_t-left_t)
                result.append(left_v + fraction*(right_v-left_v))
                break
    return result


def plot_open_closed(open_csv: PathLike, closed_csv: PathLike, signal: str, output: PathLike,
                     alignment: str = "exact") -> Path:
    open_rows, closed_rows = load_csv(open_csv), load_csv(closed_csv)
    open_name, closed_name = _channel(open_rows, signal), _channel(closed_rows, signal)
    _validate_rows(open_rows, [open_name])
    _validate_rows(closed_rows, [closed_name])
    open_times = [row["time_s"] for row in open_rows]
    closed_times = [row["time_s"] for row in closed_rows]
    if alignment == "exact":
        if open_times != closed_times:
            raise ValueError("open/closed time grids differ; select alignment='linear' explicitly")
        closed_values = [row[closed_name] for row in closed_rows]
    elif alignment == "linear":
        closed_values = _interpolate(closed_rows, closed_name, open_times)
    else:
        raise ValueError("alignment must be 'exact' or the documented 'linear' policy")
    if _unit_for(open_name) != _unit_for(closed_name) or _frame_for(open_name) != _frame_for(closed_name):
        raise ValueError("open/closed channels have incompatible units or frames")
    rows = [{"time_s": row["time_s"], "open": row[open_name], "closed": value}
            for row, value in zip(open_rows, closed_values)]
    return plot_time_history_from_rows(rows, ["open", "closed"], output,
                                       f"open/closed {signal} [{_unit_for(open_name)}]")


def plot_ensemble_member_values(values: Sequence[float], output: PathLike,
                                member_ids: Optional[Sequence[str]] = None,
                                statuses: Optional[Sequence[str]] = None,
                                included: Optional[Sequence[bool]] = None,
                                denominator: Optional[int] = None,
                                title: Optional[str] = None) -> Path:
    if not values:
        raise ValueError("ensemble member plot requires values")
    count = len(values)
    if member_ids is not None and len(member_ids) != count:
        raise ValueError("member_ids must match values")
    if statuses is not None and len(statuses) != count:
        raise ValueError("statuses must match values")
    if included is not None and len(included) != count:
        raise ValueError("included must match values")
    if denominator is None:
        denominator = sum(included) if included is not None else count
    rows = [{"time_s": float(index), "value": value} for index, value in enumerate(values)]
    labels = member_ids or [str(index) for index in range(count)]
    title = title or "ensemble member values (not a population distribution)"
    title = f"{title}; included={denominator}/{count}"
    markers: Dict[str, List[float]] = {}
    if statuses:
        markers["failed/excluded"] = [float(i) for i, status in enumerate(statuses)
                                       if status not in ("completed", "passed")]
    if included:
        markers["excluded from statistic"] = [float(i) for i, keep in enumerate(included) if not keep]
    target = plot_time_history_from_rows(rows, ["value"], output, title, markers)
    text = target.read_text()
    target.write_text(text.replace("time (s)", "member index"))
    return target


def plot_ensemble_distribution(values: Sequence[float], output: PathLike,
                               title: str = "ensemble member values (not a population distribution)",
                               statuses: Optional[Sequence[str]] = None) -> Path:
    return plot_ensemble_member_values(values, output, statuses=statuses, title=title)


def plot_sweep(parameter_values: Sequence[float], response_values: Sequence[float],
               output: PathLike, parameter_name: str, response_name: str = "response",
               parameter_unit: str = "1", response_unit: str = "1") -> Path:
    if not parameter_values or len(parameter_values) != len(response_values):
        raise ValueError("sweep parameter and response sequences must have equal non-zero length")
    width, height, left, right, top, bottom = 960, 520, 70, 30, 55, 65
    x = _scale(list(parameter_values), min(parameter_values), max(parameter_values),
               left, width-left-right)
    y = _scale(list(response_values), min(response_values), max(response_values),
               height-bottom, -(height-bottom-top))
    svg = _svg_frame(f"{response_name} versus {parameter_name}", width, height)
    svg.append(f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}"/>')
    svg.append(f'<line class="axis" x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}"/>')
    svg.append(f'<polyline class="trace" points="{" ".join(f"{a:.2f},{b:.2f}" for a,b in zip(x,y))}" stroke="#086788"/>')
    svg.append(f'<text x="{width/2:.0f}" y="{height-15}" text-anchor="middle">{html.escape(parameter_name)} [{html.escape(parameter_unit)}]</text>')
    svg.append(f'<text x="12" y="{top+10}">{max(response_values):.5g} {html.escape(response_unit)}</text>')
    svg.append(f'<text x="12" y="{height-bottom}">{min(response_values):.5g} {html.escape(response_unit)}</text>')
    svg.append("</svg>")
    target = Path(output)
    target.write_text("\n".join(svg))
    return target


def export_png(svg_source: PathLike, png_output: PathLike) -> Path:
    """Rasterize SVG offline through the optional ``png`` extra."""
    try:
        import cairosvg
    except ImportError as error:  # pragma: no cover - environment dependent
        raise RuntimeError("PNG export requires: pip install 'galata-engineering[png]'") from error
    target = Path(png_output)
    cairosvg.svg2png(url=str(svg_source), write_to=str(target))
    return target


def plot_trajectory_projection(csv_path: PathLike, output: PathLike) -> Path:
    """Static NED-to-display projection; this function is intentionally not playback."""
    rows = load_csv(csv_path)
    _validate_rows(rows, ["position_north_m", "position_east_m", "position_down_m"])
    north = [row["position_north_m"] for row in rows]
    east = [row["position_east_m"] for row in rows]
    up = [-row["position_down_m"] for row in rows]
    if any(value is None or not math.isfinite(value) for value in north + east + up):
        raise ValueError("trajectory projection requires finite position observations")
    projected_x = [n + 0.45 * e for n, e in zip(north, east)]
    projected_y = [u + 0.25 * e for u, e in zip(up, east)]
    left, right, top, bottom, width, height = 90.0, 40.0, 70.0, 80.0, 960.0, 520.0
    low, high = min(projected_x + projected_y), max(projected_x + projected_y)
    x = _scale(projected_x, min(projected_x), max(projected_x), left, width-left-right)
    y = _scale(projected_y, low, high, height-bottom, -(height-bottom-top))
    svg = _svg_frame("3-D trajectory projection (N/E/up; source NED)", int(width), int(height))
    svg.append(f'<polyline class="trace" points="{" ".join(f"{a:.2f},{b:.2f}" for a,b in zip(x,y))}" stroke="#086788"/>')
    svg.append(f'<text x="{left}" y="{height-25}">display: N/E/up; up = -NED down</text>')
    svg.append("</svg>")
    target = Path(output)
    target.write_text("\n".join(svg))
    return target


def plot_trajectory_playback(csv_path: PathLike, output: PathLike) -> Path:
    """Write a self-contained offline HTML scrubber using recorded attitude."""
    rows = load_csv(csv_path)
    _validate_rows(rows, ["position_north_m", "position_east_m", "position_down_m",
                          "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z"])
    frames = [{"time_s": row["time_s"], "north_m": row["position_north_m"],
               "east_m": row["position_east_m"], "down_m": row["position_down_m"],
               "quaternion": [row["quaternion_w"], row["quaternion_x"],
                              row["quaternion_y"], row["quaternion_z"]]}
              for row in rows]
    for index, frame in enumerate(frames):
        values = frame["quaternion"]
        if any(value is None or not math.isfinite(value) for value in values):
            raise ValueError(f"quaternion is missing or non-finite at row {index}")
        norm = math.sqrt(sum(value * value for value in values))
        if abs(norm - 1.0) > 1e-6:
            raise ValueError(f"quaternion is not normalized at row {index}: norm={norm}")
    payload = json.dumps(frames, separators=(",", ":"))
    document = f"""<!doctype html>
<meta charset="utf-8"><title>Galata trajectory playback</title>
<style>body{{font-family:system-ui,sans-serif}} canvas{{border:1px solid #789;width:720px;height:420px}} </style>
<h1>Trajectory playback (recorded body-to-NED attitude)</h1>
<p>NED source is displayed as North/East/up with <code>up = -down</code>. The body-forward
vector is rotated by the recorded Hamilton scalar-first quaternion.</p>
<input id="scrub" type="range" min="0" max="{len(frames)-1}" value="0" step="1">
<output id="time"></output><canvas id="view" width="720" height="420"></canvas>
<script>
const frames={payload}; const scrub=document.querySelector('#scrub');
const time=document.querySelector('#time'); const canvas=document.querySelector('#view');
const ctx=canvas.getContext('2d');
function rotate(q,v) {{ const [w,x,y,z]=q; const t=[2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])];
 return [v[0]+w*t[0]+(y*t[2]-z*t[1]),v[1]+w*t[1]+(z*t[0]-x*t[2]),v[2]+w*t[2]+(x*t[1]-y*t[0])]; }}
function draw() {{ const f=frames[Number(scrub.value)]; ctx.clearRect(0,0,720,420); ctx.strokeStyle='#086788';
 ctx.beginPath(); frames.forEach((p,i)=>{{const x=60+p.north_m*4+0.7*p.east_m; const y=330-p.down_m*4;
 i?ctx.lineTo(x,y):ctx.moveTo(x,y);}}); ctx.stroke(); const x=60+f.north_m*4+0.7*f.east_m, y=330-f.down_m*4;
 const forward=rotate(f.quaternion,[1,0,0]); ctx.fillStyle='#d1495b'; ctx.beginPath(); ctx.arc(x,y,7,0,Math.PI*2);ctx.fill();
 ctx.strokeStyle='#d1495b';ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+35*forward[0],y-35*forward[2]);ctx.stroke();
 time.value=`t=${{f.time_s}} s, q=[${{f.quaternion.join(', ')}}], body-forward NED=[${{forward.map(v=>v.toFixed(4)).join(', ')}}]`; }}
scrub.addEventListener('input',draw); draw();
</script>"""
    target = Path(output)
    target.write_text(document)
    return target
