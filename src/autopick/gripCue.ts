/**
 * What two marked points mean for the gripper, worked out instantly in the browser so it can be drawn as
 * soon as the second point is marked. Same rules as classify_points in jenga_mujoco/grasp_planner.py, which
 * has the final say: change them together.
 */
import * as THREE from 'three';
import type { MarkedPoint } from './api';

const FRICTION = 0.5;
const LIMIT_DEG = 2 * THREE.MathUtils.radToDeg(Math.atan(FRICTION));
const OPEN_MARGIN_MM = 4;

export interface GripCue {
  ok: boolean;
  text: string;
  /** Only when the surfaces define a closing direction. */
  geometry?: {
    mode: 'outside' | 'inside';
    axis: THREE.Vector3; // closing axis, part frame
    center: THREE.Vector3; // midpoint of the two points
    widthMm: number; // between the two finger planes
    offsetMm: number; // how far the points are out of line with each other
  };
}

const vec = (v: { x: number; y: number; z: number }) => new THREE.Vector3(v.x, v.y, v.z);

export const describeGrip = (points: MarkedPoint[], maxOpeningMm: number): GripCue | null => {
  if (points.length !== 2) return null;
  const [p1, p2] = points.map((m) => vec(m.point));
  const [n1, n2] = points.map((m) => vec(m.normal).normalize());

  if (p1.distanceTo(p2) < 1) return { ok: false, text: 'The two points are in the same place.' };
  const opposition = -n1.dot(n2);
  if (opposition <= 0) {
    return { ok: false, text: 'These surfaces face the same way: the fingers cannot squeeze between them.' };
  }
  const apartDeg = THREE.MathUtils.radToDeg(Math.acos(Math.min(1, opposition)));

  const axis = n1.clone().sub(n2).normalize();
  const between = p1.clone().sub(p2);
  const across = between.dot(axis);
  const widthMm = Math.abs(across);
  if (widthMm < 1) return { ok: false, text: 'The points are side by side, with no thickness between them to grip.' };

  const mode = across > 0 ? 'outside' : 'inside';
  const geometry = {
    mode,
    axis,
    center: p1.clone().add(p2).multiplyScalar(0.5),
    widthMm,
    offsetMm: between.addScaledVector(axis, -across).length()
  } as const;

  if (apartDeg > LIMIT_DEG) {
    return {
      ok: false,
      geometry,
      text: `These surfaces are ${apartDeg.toFixed(0)}° from facing opposite (limit ${LIMIT_DEG.toFixed(0)}°): the fingers would slide off.`
    };
  }
  if (mode === 'outside' && widthMm > maxOpeningMm - OPEN_MARGIN_MM) {
    return {
      ok: false,
      geometry,
      text: `${widthMm.toFixed(1)} mm between the points, but this gripper opens only ${maxOpeningMm.toFixed(0)} mm.`
    };
  }
  const action = mode === 'outside' ? 'Fingers close on the points' : 'Fingers enter closed, then open against the points';
  return {
    ok: true,
    geometry,
    text: `${action}: ${widthMm.toFixed(1)} mm between them, surfaces ${apartDeg.toFixed(0)}° from opposite.`
  };
};
