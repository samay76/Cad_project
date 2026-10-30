"""
cad_part.py - Adds a converted CAD part (see step_to_mesh.py) to an MjSpec.

Body frame == the part's CAD frame (meters). Visual mesh is display-only
(contype/conaffinity 0); collision uses the CoACD convex hulls. Mass and
inertia come from the exact B-rep volume (steel by default), not the hulls.
"""
import os

import mujoco

G = mujoco.mjtGeom

PART_BODY = 'cad_part'
PART_JOINT = 'cad_part_free'

# Steel-on-silicone pad contact. priority=2 > gripper pads' priority=1, so these
# params are the ones MuJoCo uses for pad<->part contacts. condim 4 adds
# torsional friction, which keeps the part from spinning about the pad normal.
PART_FRICTION = [1.0, 0.01, 0.0002]
PART_CONDIM = 4
PART_PRIORITY = 2


def add_cad_part(spec, meta, pos, quat, free=True, rgba=(0.72, 0.74, 0.78, 1.0), hull_alpha=0.0,
                 name=PART_BODY):
    """Adds the part body. `quat` is MuJoCo order (w, x, y, z)."""
    d = meta['dir']
    vis_mesh = spec.add_mesh(name=f'{name}_visual', file=os.path.join(d, meta['visual_mesh']))
    vis_mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL  # visual mesh need not be watertight

    body = spec.worldbody.add_body(name=name, pos=list(pos), quat=list(quat))
    if free:
        body.add_freejoint(name=PART_JOINT)

    body.explicitinertial = True
    body.mass = meta['mass_kg']
    body.ipos = meta['com_m']
    body.fullinertia = meta['fullinertia']

    body.add_geom(name=f'{name}_visual', type=G.mjGEOM_MESH, meshname=vis_mesh.name,
                  contype=0, conaffinity=0, group=1, rgba=list(rgba), mass=0)

    for i, hull_file in enumerate(meta['collision_hulls']):
        m = spec.add_mesh(name=f'{name}_hull_{i:02d}', file=os.path.join(d, hull_file))
        body.add_geom(name=f'{name}_hull_{i:02d}', type=G.mjGEOM_MESH, meshname=m.name,
                      group=3, mass=0, friction=PART_FRICTION, condim=PART_CONDIM, priority=PART_PRIORITY,
                      rgba=[0.2, 0.6, 1.0, hull_alpha])
    return body
