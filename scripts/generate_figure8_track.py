#!/usr/bin/env python3
"""Generate a visual-only SDF Bernoulli-lemniscate line model.

A lemniscate (r^2 = a^2 cos(2*theta) in polar form) is used instead of two
externally-tangent circles because the two lobes cross at a genuine ~90
degree angle at the origin, not a shared tangent line. A vision-based line
follower that always steers toward the least-deviating contour treats a
tangent crossing as one continuous straight line (so it just keeps
completing the same loop forever); a real angled crossing gives it two
visually distinct directions to choose between, so "keep going straight"
naturally carries it into the other lobe instead.
"""

import argparse
import math


def _lobe_points(scale, phase, segments):
    """Sample one lemniscate lobe: closed loop through the origin.

    ``phase`` is the polar angle of the lobe's own axis (0 for the lobe
    lying along +x, pi for the one along -x). theta sweeps
    phase-pi/4 .. phase+pi/4, where r = scale*sqrt(cos(2*(theta-phase)))
    is exactly zero at both ends - the lobe starts and ends at the origin,
    which is what makes the two lobes share that single crossing point.
    """
    points = []
    half_span = math.pi / 4.0
    for i in range(segments + 1):
        local_theta = -half_span + i * (2.0 * half_span) / segments
        # Make the shared crossing exact.  On some libm implementations,
        # cos(+/-pi/2) is a tiny positive or negative value rather than zero.
        # The clamp also safely absorbs the latter case before sqrt().
        radial_term = 0.0 if i in (0, segments) else max(
            0.0, math.cos(2.0 * local_theta)
        )
        r = scale * math.sqrt(radial_term)
        theta = phase + local_theta
        points.append((r * math.cos(theta), r * math.sin(theta)))
    return points


def _visuals(scale, line_width, thickness, segments_per_loop):
    """Yield one SDF visual block for each polyline segment."""
    visual_index = 0

    for phase in (0.0, math.pi):
        points = _lobe_points(scale, phase, segments_per_loop)
        for p0, p1 in zip(points, points[1:]):
            mid_x = (p0[0] + p1[0]) / 2.0
            mid_y = (p0[1] + p1[1]) / 2.0
            length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

            yield (
                f'      <visual name="segment_{visual_index:03d}">\n'
                f'        <pose>{mid_x:.9f} {mid_y:.9f} '
                f'{thickness / 2.0:.9f} 0 0 {yaw:.9f}</pose>\n'
                '        <geometry>\n'
                f'          <box><size>{length:.9f} {line_width:.9f} '
                f'{thickness:.9f}</size></box>\n'
                '        </geometry>\n'
                '        <material>\n'
                '          <ambient>0.02 0.02 0.02 1</ambient>\n'
                '          <diffuse>0.03 0.03 0.03 1</diffuse>\n'
                '          <specular>0.01 0.01 0.01 1</specular>\n'
                '        </material>\n'
                '      </visual>'
            )
            visual_index += 1


def main():
    parser = argparse.ArgumentParser(
        description="Print a visual-only SDF lemniscate (figure-eight) line model."
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.5,
        help="Distance from the origin to each lobe's widest point (metres).",
    )
    parser.add_argument("--line-width", type=float, default=0.03)
    parser.add_argument("--thickness", type=float, default=0.002)
    parser.add_argument("--segments-per-loop", type=int, default=16)
    parser.add_argument("--model-name", default="figure8_line")
    args = parser.parse_args()

    if not math.isfinite(args.scale) or args.scale <= 0.0:
        parser.error("--scale must be positive")
    if not math.isfinite(args.line_width) or args.line_width <= 0.0:
        parser.error("--line-width must be positive")
    if not math.isfinite(args.thickness) or args.thickness <= 0.0:
        parser.error("--thickness must be positive")
    if args.segments_per_loop < 8:
        parser.error("--segments-per-loop must be at least 8")

    print(f'<model name="{args.model_name}">')
    print("  <static>true</static>")
    print('  <link name="link">')
    for visual in _visuals(
        args.scale, args.line_width, args.thickness, args.segments_per_loop
    ):
        print(visual)
    print("  </link>")
    print("</model>")


if __name__ == "__main__":
    main()
