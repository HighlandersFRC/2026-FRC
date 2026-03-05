"""
flip_auto.py - Flip a .polarauto path file across the field's Y-axis midline.

Transformation applied to every point (key_points and points):
  - x         stays the same
  - y         -> FIELD_WIDTH - y
  - angle     -> -angle  (negate theta)
  - y_velocity        -> -y_velocity
  - angular_velocity  -> -angular_velocity
  - y_acceleration    -> -y_acceleration
  - angular_acceleration -> -angular_acceleration
  (x_velocity, x_acceleration, and time are unchanged)

Usage:
  python flip_auto.py <input_file> [output_file]

If output_file is omitted the result is written next to the input file
with " (flipped)" appended to the stem, e.g.:
  "L sweep n climb.polarauto"  ->  "L sweep n climb (flipped).polarauto"
"""

import json
import sys
import os

# FRC 2026 field width in metres (the Y dimension when X is the long axis)
FIELD_WIDTH = 8.069


def flip_point(pt: dict) -> dict:
    """Return a new point dict with the Y-flip transformation applied."""
    flipped = dict(pt)  # shallow copy – all values are scalars
    flipped["y"] = FIELD_WIDTH - pt["y"]
    flipped["angle"] = -pt["angle"]
    flipped["y_velocity"] = -pt["y_velocity"]
    flipped["angular_velocity"] = -pt["angular_velocity"]
    flipped["y_acceleration"] = -pt["y_acceleration"]
    flipped["angular_acceleration"] = -pt["angular_acceleration"]
    return flipped


def flip_auto(data: dict) -> dict:
    """Flip all paths in the parsed .polarauto JSON."""
    import copy
    result = copy.deepcopy(data)

    for path in result.get("paths", []):
        # Flip key_points (the user-defined waypoints)
        path["key_points"] = [flip_point(p) for p in path.get("key_points", [])]

        # Flip the dense sampled points array (if present)
        if "points" in path:
            path["points"] = [flip_point(p) for p in path["points"]]

    return result


def main():
    if len(sys.argv) < 2:
        print("Usage: python flip_auto.py <input_file> [output_file]")
        sys.exit(1)

    input_path = sys.argv[1]

    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        stem, ext = os.path.splitext(input_path)
        output_path = f"{stem} (flipped){ext}"

    with open(input_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    flipped = flip_auto(data)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(flipped, f, separators=(",", ":"))

    print(f"Flipped auto written to: {output_path}")


if __name__ == "__main__":
    main()
