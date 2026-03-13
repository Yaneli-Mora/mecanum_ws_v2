#!/usr/bin/env python3
# generate_arena_map.py — generates arena_map.pgm and arena_map.yaml
#
# USAGE:
#   python3 generate_arena_map.py
#
# ⚠️  MEASURE ALL VALUES BELOW ON YOUR PHYSICAL FIELD BEFORE COMPETITION

from PIL import Image, ImageDraw
import numpy as np

RESOLUTION   = 0.01   # meters per pixel

# ══════════════════════════════════════════════════════════════════════════
# FIELD — measure inside wall to inside wall
# ══════════════════════════════════════════════════════════════════════════
FIELD_WIDTH  = 2.43   # meters (x, left → right)
FIELD_HEIGHT = 1.21   # meters (y, bottom → top)
WALL_THICKNESS = 2    # pixels

# ══════════════════════════════════════════════════════════════════════════
# CRATER — measure center from bottom-left corner, and radius
# ══════════════════════════════════════════════════════════════════════════
CRATER_X      = 1.50  # ⚠️ MEASURE ON FIELD
CRATER_Y      = 0.59  # ⚠️ MEASURE ON FIELD
CRATER_RADIUS = 0.30  # ⚠️ MEASURE ON FIELD

# ══════════════════════════════════════════════════════════════════════════
# ANTENNA BOXES — measure center of each box from bottom-left corner
# WIDTH and HEIGHT are the physical box dimensions
# Add a small margin so Nav2 inflation doesn't cut it too close
# ══════════════════════════════════════════════════════════════════════════
ANTENNA_BOXES = [
    # (center_x, center_y, width, height)  all in meters
    (0.1397, 1.0795, 0.10, 0.10),   # antenna_1 ⚠️ MEASURE
    (2.2987, 1.0795, 0.10, 0.10),   # antenna_2 ⚠️ MEASURE
    (1.2003, 2.2479, 0.10, 0.10),   # antenna_3 ⚠️ MEASURE
    (2., 0.1397, 0.10, 0.10),   # antenna_4 ⚠️ MEASURE
]

# Output
OUTPUT_PGM  = "arena_map.pgm"
OUTPUT_YAML = "arena_map.yaml"

# ══════════════════════════════════════════════════════════════════════════

def world_to_pixel(x_m, y_m, height_px):
    px = int(round(x_m / RESOLUTION))
    py = height_px - 1 - int(round(y_m / RESOLUTION))
    return px, py

def generate():
    width_px  = int(round(FIELD_WIDTH  / RESOLUTION))
    height_px = int(round(FIELD_HEIGHT / RESOLUTION))

    print(f"Arena: {FIELD_WIDTH}m x {FIELD_HEIGHT}m → {width_px}x{height_px}px")

    arr = np.full((height_px, width_px), 254, dtype=np.uint8)
    img = Image.fromarray(arr, mode='L')
    draw = ImageDraw.Draw(img)

    # ── Outer walls ───────────────────────────────────────────────────────
    for t in range(WALL_THICKNESS):
        draw.rectangle([t, t, width_px-1-t, height_px-1-t], outline=0)

    # ── Crater ────────────────────────────────────────────────────────────
    cx, cy = world_to_pixel(CRATER_X, CRATER_Y, height_px)
    r = int(round(CRATER_RADIUS / RESOLUTION))
    draw.ellipse([cx-r, cy-r, cx+r, cy+r], fill=0)
    print(f"Crater: center=({CRATER_X},{CRATER_Y}) r={CRATER_RADIUS}m at pixel ({cx},{cy})")

    # ── Antenna boxes ─────────────────────────────────────────────────────
    for i, (bx, by, bw, bh) in enumerate(ANTENNA_BOXES):
        cx, cy = world_to_pixel(bx, by, height_px)
        hw = int(round((bw / 2) / RESOLUTION))
        hh = int(round((bh / 2) / RESOLUTION))
        draw.rectangle([cx-hw, cy-hh, cx+hw, cy+hh], fill=0)
        print(f"Antenna {i+1}: center=({bx},{by}) size={bw}x{bh}m at pixel ({cx},{cy})")

    # ── Save PGM ──────────────────────────────────────────────────────────
    img.save(OUTPUT_PGM)
    print(f"\nSaved: {OUTPUT_PGM}")

    # ── Save YAML ─────────────────────────────────────────────────────────
    with open(OUTPUT_YAML, 'w') as f:
        f.write(f"""image:      {OUTPUT_PGM}
resolution: {RESOLUTION}
origin:     [0.0, 0.0, 0.0]
negate:     0
occupied_thresh:  0.65
free_thresh:      0.196
""")
    print(f"Saved: {OUTPUT_YAML}")

    result = np.array(img)
    print(f"\nFree pixels: {np.sum(result == 254)}")
    print(f"Obstacle pixels: {np.sum(result == 0)}")
    print(f"\n⚠️  Remember to measure and update:")
    print(f"   CRATER_X, CRATER_Y, CRATER_RADIUS")
    print(f"   Each ANTENNA_BOXES entry")
    print(f"\nCopy both files to: ~/mecanum_ws_v2/src/mecanum_bringup/map/")

if __name__ == "__main__":
    generate()
