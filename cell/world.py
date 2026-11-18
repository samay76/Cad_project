"""
world.py - The cell around the robot: table (Table_base.step), pick fixture, target tray, tape and light.
The table top is at z = 0; the robot stands on the table's mounting plate (table_base.MOUNT_PLATE_TOP_Z).
"""
import mujoco

from .table_base import add_table

G = mujoco.mjtGeom

TRAY_XY = (0.55, 0.15)       # default tray centre (config/cell.json place_xy)
TRAY_HALF = (0.065, 0.050)   # inner half-size of the target tray (m); bigger parts get a bigger one


def add_world(spec, tray_xy=TRAY_XY, tray_half=TRAY_HALF, pick_station=None):
    """Table, pick fixture (pick_station: (info, placement, seat_part) from pick_station.from_config), the target
    tray with walls at tray_xy and a yellow tape boundary around it, and the scene light."""
    w = spec.worldbody
    add_table(spec)
    if pick_station is not None:
        from .pick_station import add_to_spec
        add_to_spec(spec, *pick_station)

    bx, by = (float(v) for v in tray_xy)
    bw, bl = tray_half
    w.add_geom(name='boundary', type=G.mjGEOM_BOX, size=[bw + 0.025, bl + 0.020, 0.0005],
               pos=[bx, by, 0.0005], rgba=[1.0, 0.88, 0.15, 1], contype=0, conaffinity=0)
    bh, wall_th, wall_h = 0.003, 0.003, 0.012
    w.add_geom(name='box_bottom', type=G.mjGEOM_BOX, size=[bw, bl, bh],
               pos=[bx, by, bh], rgba=[0.22, 0.26, 0.34, 1.0], friction=[1.0, 0.005, 0.0001])
    wall = [0.28, 0.33, 0.42, 1.0]
    w.add_geom(name='box_wall_left', type=G.mjGEOM_BOX, size=[wall_th, bl, wall_h], pos=[bx - bw, by, wall_h], rgba=wall)
    w.add_geom(name='box_wall_right', type=G.mjGEOM_BOX, size=[wall_th, bl, wall_h], pos=[bx + bw, by, wall_h], rgba=wall)
    w.add_geom(name='box_wall_front', type=G.mjGEOM_BOX, size=[bw, wall_th, wall_h], pos=[bx, by - bl, wall_h], rgba=wall)
    w.add_geom(name='box_wall_back', type=G.mjGEOM_BOX, size=[bw, wall_th, wall_h], pos=[bx, by + bl, wall_h], rgba=wall)

    w.add_light(pos=[0.3, 0, 2], dir=[0, 0, -1])
