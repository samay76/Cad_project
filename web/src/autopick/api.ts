/**
 * Types and requests for the automatic pick & place (vite-plugins/mujocoBridge.ts, cell/auto_pick_place.py).
 * All poses are in the part's starting frame, in mm, with quaternions in xyzw order.
 */
import type { Pose, SavedGrasp, Vector3D } from '../types';

export interface PlanPhase {
  name: string;
  label: string;
  width: number; // finger opening at the end of the phase, mm
  attached: boolean; // the part is carried during this phase
  seconds: number;
  tcp: Pose[];
}

export interface GraspPlan {
  grasp: SavedGrasp;
  mode: 'outside' | 'inside';
  widthMm: number;
  approachTiltDeg: number;
  score: number;
  durationS: number;
  graspTest?: boolean; // config grasp_test: go, close, hold, open, home (no lift / place)
  touch?: { pointMm: number[]; standoffMm: number; rollDeg: number }; // touch test: fingertips to one point
  T_tcp_part: number[][];
  phases: PlanPhase[];
}

export interface PlanDocument {
  robot?: string; // robot id the plan was made for ('ec66')
  robotLabel: string;
  gripperLabel: string;
  awayFromClick: boolean;
  part: { T_world_part: number[][]; tippedDeg: number; driftMm: number; massKg: number; sizeMm: number[] };
  scene: { boxes: { name: string; pos: number[]; halfSize: number[] }[]; placeXY: number[] };
  world_in_part: Pose;
  plans: GraspPlan[];
  seconds: number;
}

export interface SimStatus {
  state: 'running' | 'done' | 'failed';
  phase?: string;
  message?: string;
  ok?: boolean | null;
  info?: Record<string, unknown>;
}

/** A point the user marked for a finger, in the part frame (mm) with its surface normal. */
export interface MarkedPoint {
  point: Vector3D;
  normal: Vector3D;
}

export interface PlanRequest {
  partFileName: string;
  partFileBase64: string;
  partWorldPose: Pose;
  robotId: string;
  gripperId: string;
  tcpOffset: number;
  preGraspDistance: number;
  points: MarkedPoint[]; // grip: one per finger; touch: the one point the fingertips go to
  mode: 'grip' | 'touch';
  pickOffsetMm: { x: number; y: number; z: number }; // added to the grasp position, robot base frame
  graspPose: Pose | null; // gripper pose set in the view (part frame, mm): planned exactly when given
}

export const arrayBufferToBase64 = (buf: ArrayBuffer): string => {
  const bytes = new Uint8Array(buf);
  let binary = '';
  for (let i = 0; i < bytes.length; i += 0x8000) {
    binary += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  }
  return btoa(binary);
};

const post = async (url: string, body: unknown): Promise<string> => {
  const resp = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body)
  });
  const res = await resp.json();
  if (!resp.ok || !res.ok) throw new Error(res.message || `HTTP ${resp.status}`);
  return res.runId;
};

export const startPlan = (request: PlanRequest) => post('/api/mujoco/auto/plan', request);

export const startRun = (runId: string, candidate: number, mode: 'preview' | 'execute') =>
  post('/api/mujoco/auto/run', { runId, candidate, mode });

export const fetchPlan = async (runId: string): Promise<PlanDocument> => {
  const resp = await fetch(`/api/mujoco/auto/plan?id=${encodeURIComponent(runId)}`);
  if (!resp.ok) throw new Error(`The plan could not be read (HTTP ${resp.status}).`);
  return resp.json();
};

export const fetchStatus = async (runId: string): Promise<SimStatus> => {
  const resp = await fetch(`/api/mujoco/status?id=${encodeURIComponent(runId)}`);
  return resp.json();
};

/** Real robot connection for "Real Sim" (cell/real_robot.py). ip 'fake' = simulated robot. */
export interface RealRobotSettings {
  ip: string;
  src: string; // local IP to connect from, optional
  speed: number; // joint speed %, 1..10
  gripper: boolean; // drive the real Robotiq 2F-85 on USB during Real Sim
}

export const startRealRun = (runId: string, candidate: number, robot: RealRobotSettings, action: 'real' | 'home' = 'real') =>
  post('/api/mujoco/auto/real', {
    runId,
    candidate,
    robotIp: robot.ip,
    robotSrc: robot.src,
    speed: robot.speed,
    gripper: robot.gripper,
    action
  });

/** Stops the running simulator process; during Real Sim this also sends `stop` to the robot. */
export const stopRun = async () => {
  await fetch('/api/mujoco/stop', { method: 'POST' });
};
