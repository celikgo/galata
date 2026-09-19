"""Dependency-light offline SVG plots and optional PNG rasterization."""

from __future__ import annotations

import html
from pathlib import Path
from typing import List, Mapping, Optional, Sequence, Union

from .workflow import load_csv


def _scale(values: Sequence[float], lo: float, hi: float, start: float, size: float) -> List[float]:
    span = hi - lo if hi > lo else 1.0
    return [start + (value - lo) * size / span for value in values]


def _svg_frame(title: str, width: int = 960, height: int = 520) -> List[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">',
        "<style>text{font-family:system-ui,sans-serif;fill:#243447} "
        ".axis{stroke:#496273;stroke-width:1}.trace{fill:none;stroke-width:2}</style>",
        f'<text x="60" y="30" font-size="18">{html.escape(title)}</text>',
    ]


def _time_plot(rows: Sequence[Mapping[str, float]], channels: Sequence[str], title: str,
               markers: Optional[Mapping[str, Sequence[float]]] = None) -> str:
    width, height, left, right, top, bottom = 960, 520, 70, 30, 55, 65
    time = [row["time_s"] for row in rows]
    values = [row[name] for name in channels for row in rows]
    x = _scale(time, min(time), max(time), left, width - left - right)
    output = _svg_frame(title, width, height)
    output.append(f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}"/>')
    output.append(f'<line class="axis" x1="{left}" y1="{height-bottom}" '
                  f'x2="{width-right}" y2="{height-bottom}"/>')
    output.append(f'<text x="{width/2:.0f}" y="{height-15}" text-anchor="middle">time (s)</text>')
    if markers:
        marker_colours = ("#6a4c93", "#1982c4", "#8ac926", "#ff595e")
        for marker_index, (label, times) in enumerate(markers.items()):
            colour = marker_colours[marker_index % len(marker_colours)]
            for marker_time in times:
                marker_x = _scale([marker_time], min(time), max(time),
                                  left, width - left - right)[0]
                output.append(
                    f'<line x1="{marker_x:.2f}" y1="{top}" x2="{marker_x:.2f}" '
                    f'y2="{height-bottom}" stroke="{colour}" stroke-dasharray="4,3"/>')
            output.append(f'<text x="{left+10+marker_index*160}" y="{top-12}" fill="{colour}">'
                          f'{html.escape(label)}</text>')
    low, high = min(values), max(values)
    colours = ("#086788", "#d1495b", "#2a9d8f", "#f29e4c")
    for index, name in enumerate(channels):
        y = _scale([row[name] for row in rows], low, high, height-bottom, -(height-bottom-top))
        points = " ".join(f"{xx:.2f},{yy:.2f}" for xx, yy in zip(x, y))
        colour = colours[index % len(colours)]
        output.append(f'<polyline class="trace" points="{points}" stroke="{colour}"/>')
        output.append(f'<text x="{left+10+index*180}" y="{height-38}" fill="{colour}">'
                      f'{html.escape(name)}</text>')
    output.append(f'<text x="12" y="{top+10}">{high:.5g}</text>')
    output.append(f'<text x="12" y="{height-bottom}">{low:.5g}</text>')
    output.append("</svg>")
    return "\n".join(output)


def plot_time_history(csv_path: Union[Path, str], channels: Sequence[str], output: Union[Path, str],
                      title: str = "Galata time history",
                      markers: Optional[Mapping[str, Sequence[float]]] = None) -> Path:
    rows = load_csv(Path(csv_path))
    if not rows:
        raise ValueError("CSV has no data rows")
    missing = [name for name in channels if name not in rows[0]]
    if missing:
        raise KeyError(f"missing named channels: {missing}")
    target = Path(output)
    target.write_text(_time_plot(rows, channels, title, markers))
    return target


def plot_time_history_from_rows(rows: Sequence[Mapping[str, float]], channels: Sequence[str],
                                output: Path, title: str,
                                markers: Optional[Mapping[str, Sequence[float]]] = None) -> Path:
    if not rows:
        raise ValueError("plot requires at least one row")
    output.write_text(_time_plot(rows, channels, title, markers))
    return output


def plot_open_closed(open_csv: Union[Path, str], closed_csv: Union[Path, str], signal: str,
                     output: Union[Path, str]) -> Path:
    open_rows, closed_rows = load_csv(Path(open_csv)), load_csv(Path(closed_csv))
    open_name = signal if signal in open_rows[0] else f"output_{signal}"
    closed_name = signal if signal in closed_rows[0] else f"output_{signal}"
    rows = [
        {"time_s": left["time_s"], "open": left[open_name], "closed": right[closed_name]}
        for left, right in zip(open_rows, closed_rows)
    ]
    return plot_time_history_from_rows(rows, ["open", "closed"], Path(output),
                                       f"open/closed {signal}")


def plot_ensemble_distribution(values: Sequence[float], output: Union[Path, str],
                               title: str = "ensemble distribution",
                               statuses: Optional[Sequence[str]] = None) -> Path:
    if not values:
        raise ValueError("ensemble distribution requires values")
    rows = [{"time_s": float(index), "value": value} for index, value in enumerate(values)]
    if statuses is not None and len(statuses) != len(values):
        raise ValueError("ensemble statuses must match values")
    marker_rows = None
    if statuses:
        failed = [float(index) for index, status in enumerate(statuses)
                  if status in ("failed", "excluded", "refused", "diverged")]
        marker_rows = {"failed/excluded": failed}
    return plot_time_history_from_rows(rows, ["value"], Path(output), title, marker_rows)


def plot_sweep(parameter_values: Sequence[float], response_values: Sequence[float],
               output: Union[Path, str], parameter_name: str,
               response_name: str = "response") -> Path:
    """Plot a named sweep curve while retaining the parameter in the rows."""
    if not parameter_values or len(parameter_values) != len(response_values):
        raise ValueError("sweep parameter and response sequences must have equal non-zero length")
    rows = [{"time_s": parameter, response_name: response}
            for parameter, response in zip(parameter_values, response_values)]
    return plot_time_history_from_rows(rows, [response_name], Path(output),
                                       f"{response_name} versus {parameter_name}")


def export_png(svg_source: Union[Path, str], png_output: Union[Path, str]) -> Path:
    """Rasterize SVG offline through the optional cairosvg dependency."""
    try:
        import cairosvg
    except ImportError as error:  # pragma: no cover - environment dependent
        raise RuntimeError("PNG export requires: pip install 'galata-engineering[png]'") from error
    target = Path(png_output)
    cairosvg.svg2png(url=str(svg_source), write_to=str(target))
    return target


def plot_trajectory_playback(csv_path: Union[Path, str], output: Union[Path, str]) -> Path:
    """Static isometric 3-D playback frame: NED source projected to N/E/up."""
    rows = load_csv(Path(csv_path))
    if not rows:
        raise ValueError("trajectory has no data rows")
    for name in ("position_north_m", "position_east_m", "position_down_m"):
        if name not in rows[0]:
            raise KeyError(name)
    north = [row["position_north_m"] for row in rows]
    east = [row["position_east_m"] for row in rows]
    up = [-row["position_down_m"] for row in rows]
    projected_x = [n + 0.45 * e for n, e in zip(north, east)]
    projected_y = [u + 0.25 * e for u, e in zip(up, east)]
    left, right, top, bottom = 90.0, 40.0, 70.0, 80.0
    width, height = 960.0, 520.0
    low = min(projected_x + projected_y)
    high = max(projected_x + projected_y)
    x = _scale(projected_x, min(projected_x), max(projected_x), left, width-left-right)
    y = _scale(projected_y, low, high, height-bottom, -(height-bottom-top))
    target = Path(output)
    svg = _svg_frame("3-D trajectory playback (N/E/up; source NED)", int(width), int(height))
    svg.append(f'<line class="axis" x1="{left}" y1="{height-bottom}" '
               f'x2="{width-right}" y2="{height-bottom}"/>')
    svg.append(f'<line class="axis" x1="{left}" y1="{height-bottom}" '
               f'x2="{left}" y2="{top}"/>')
    svg.append(f'<polyline class="trace" points="{" ".join(f"{xx:.2f},{yy:.2f}" for xx, yy in zip(x, y))}" '
               'stroke="#086788"/>')
    svg.append(f'<text x="{width-right-120}" y="{height-bottom+28}">projected N/E</text>')
    svg.append(f'<text x="{left-20}" y="{top-15}">up = -down</text>')
    svg.append(f'<text x="{left}" y="{height-25}">start t={rows[0]["time_s"]:.3g} s</text>')
    svg.append(f'<text x="{width-right-180}" y="{height-25}">end t={rows[-1]["time_s"]:.3g} s</text>')
    svg.append("</svg>")
    target.write_text("\n".join(svg))
    return target
