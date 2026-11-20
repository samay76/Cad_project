"""
config_check.py - Validates config/cell.json before the server starts: a wrong value here moves the real robot to
the wrong place, so a bad file is refused with a clear message instead of being half-used.

  python -m cell.config_check                 # checks config/cell.json (or $CELL_CONFIG)
  python -m cell.config_check other.json
"""
import os
import sys
from typing import List, Optional, Tuple

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator

from .paths import CONFIG_FILE, load_config, repo_path

XY = List[float]


class _M(BaseModel):
    model_config = ConfigDict(extra='allow')   # "_comment" keys and options the code reads with defaults


def _xy(v):
    if len(v) != 2:
        raise ValueError('needs [x, y] in metres')
    if not all(-1.5 <= c <= 1.5 for c in v):
        raise ValueError('is outside the table (|x|, |y| <= 1.5 m)')
    return v


class PickStation(_M):
    enabled: bool
    file: str
    near_xy: XY
    yaw_deg: float = 0.0
    seated_part: Optional[str] = None
    seat_all_parts: bool = False

    _near = field_validator('near_xy')(_xy)

    @field_validator('file', 'seated_part')
    @classmethod
    def _exists(cls, v):
        if v is not None and not os.path.exists(repo_path(v)):
            raise ValueError(f'file not found: {v}')
        return v


class RealGripper(_M):
    port: str
    baud: int = 115200
    slave: int = Field(9, ge=0, le=247)
    speed: int = Field(64, ge=0, le=255)
    force: int = Field(50, ge=0, le=255)
    closed_pos: int = Field(228, ge=180, le=255)   # position where the fingers touch (0 mm)


class GraspTest(_M):
    enabled: bool
    wait_before_close_s: float = Field(5.0, ge=0, le=120)
    hold_closed_s: float = Field(10.0, ge=0, le=600)


class TouchTest(_M):
    standoff_mm: float = Field(2.0, ge=0, le=50)
    hold_s: float = Field(5.0, ge=0, le=120)


class PickMotion(_M):
    home_joints_deg: dict
    orient_first: bool = True
    orient_speed_rad_s: float = Field(0.5, gt=0, le=2.0)
    travel_speed_m_s: float = Field(0.12, gt=0, le=0.5)
    approach_speed_m_s: float = Field(0.03, gt=0, le=0.2)
    grip_seconds: float = Field(1.5, gt=0, le=10)
    lift_speed_m_s: float = Field(0.04, gt=0, le=0.3)
    hold_after_grip_s: float = Field(0.0, ge=0, le=60)

    @field_validator('home_joints_deg')
    @classmethod
    def _home(cls, v):
        q = v.get('ec66')
        if q is None:
            raise ValueError('needs "ec66": [6 joint angles in degrees]')
        if len(q) != 6 or not all(isinstance(a, (int, float)) and -360 <= a <= 360 for a in q):
            raise ValueError('"ec66" must be 6 joint angles within +-360 deg')
        if not -160 <= q[2] <= 160:
            raise ValueError('"ec66" J3 must be within +-160 deg')
        return v


class Inspection(_M):
    enabled: bool = False
    camera_pos: Tuple[float, float, float] = (0.5, -0.62, 0.36)
    look_at: Tuple[float, float, float] = (0.5, -0.34, 0.20)
    fovy_deg: float = Field(45.0, gt=5, le=120)
    turn_deg: float = Field(60.0, ge=0, le=170)
    turn_speed_deg_s: float = Field(20.0, gt=0, le=90)
    hold_s: float = Field(1.0, ge=0, le=30)


class CellConfig(_M):
    pick_xy: XY
    place_xy: XY
    pick_station: PickStation
    real_gripper: RealGripper
    robot_base_offset_mm: dict
    gripper_coupling_mm: dict
    grasp_test: GraspTest
    inspection: Inspection = Inspection()
    touch_test: TouchTest = TouchTest()
    pick_motion: PickMotion
    lift_height_m: float = Field(gt=0, le=0.4)
    settle_seconds: float = Field(ge=0, le=10)
    preview_seconds: float = Field(ge=0, le=30)
    place_floor_z: float
    place_clearance_m: float = Field(ge=0, le=0.05)
    grip_force_limit: float = Field(5.0, gt=0, le=200)
    debug_pause_at_pick_s: float = Field(0.0, ge=0, le=60)
    pick_extra_depth_mm: float = Field(0.0, ge=0, le=15)

    _pick = field_validator('pick_xy')(_xy)
    _place = field_validator('place_xy')(_xy)

    @field_validator('robot_base_offset_mm')
    @classmethod
    def _offset(cls, v):
        o = v.get('ec66')
        if o is None or len(o) != 3 or not all(abs(c) <= 100 for c in o):
            raise ValueError('needs "ec66": [x, y, z] in mm, each within +-100 mm')
        return v

    @field_validator('gripper_coupling_mm')
    @classmethod
    def _coupling(cls, v):
        c = (v.get('ec66') or {}).get('2f85')
        if c is None or not 0 <= c <= 100:
            raise ValueError('needs {"ec66": {"2f85": <mm, 0..100>}}')
        return v


def check(path=None):
    """Returns a list of problems (empty = the file is good)."""
    try:
        cfg = load_config(path)
    except (OSError, ValueError) as e:
        return [f'cannot read {path or CONFIG_FILE}: {e}']
    try:
        CellConfig.model_validate(cfg)
    except ValidationError as e:
        return [f"{'.'.join(str(p) for p in err['loc'])}: {err['msg']}" for err in e.errors()]
    return []


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    problems = check(path)
    if problems:
        print(f'{path or CONFIG_FILE} has problems:\n  ' + '\n  '.join(problems))
        return 1
    print(f'{path or CONFIG_FILE}: OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
