/**
 * Types and requests for the automatic pick & place (vite-plugins/mujocoBridge.ts, jenga_mujoco/auto_pick_place.py).
 * All poses are in the part's starting frame, in mm, with quaternions in xyzw order.
 */
import type { Pose, SavedGrasp, Vector3D } from '../types';
import type { InspectionImage } from '../ui/CameraFeedPanel';

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
  inspectionViews: number;
  inspectionSkipped: string[];
  T_tcp_part: number[][];
  phases: PlanPhase[];
}

export interface PlanDocument {
  robotLabel: string;
  gripperLabel: string;
  awayFromClick: boolean;
  part: { tippedDeg: number; driftMm: number; massKg: number; sizeMm: number[] };
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
  info?: { inspection?: InspectionImage[] };
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
  points: MarkedPoint[];
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
