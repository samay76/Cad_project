/**
 * Plain-language text for the operator screens. Every message from planning, simulation and the robot goes
 * through here; the raw message always stays in Settings > Engineer log.
 *
 * Edit the tables below to change the wording. Rules: sentence case, short, no jargon, no exclamation marks.
 * Pure functions, no DOM: tested by tests/steps.test.mjs.
 */
export type Kind = 'info' | 'ok' | 'warn' | 'error';

export interface Friendly {
  text: string;
  kind: Kind;
}

/** Shown for an error nothing below matches. */
export const UNKNOWN_ERROR = 'Something went wrong. Try again or contact support.';

/**
 * [pattern, replacement, kind?]. First match wins. The replacement may be a function of the regex match.
 * kind: leave out to keep the message's own kind.
 */
type Rule = [RegExp, string | ((m: RegExpMatchArray) => string), Kind?];

const RULES: Rule[] = [
  // robot progress: "REAL robot: Lowering the part (smooth, 5 points, 3% speed)" -> "Lowering the part…"
  [/^REAL (?:robot|gripper): ([^(]+?)[\s.…]*(?:\(.*)?$/i, (m) => `${m[1].trim()}…`],
  // ---- planning
  [/^Starting the planner/i, 'Getting ready to find a grip…'],
  [/^Planning the arm path to the gripper pose you set/i, 'Finding a path to the position you set…'],
  [/Found \d+\/\d+ complete motions/i, 'Checking grip options…'],
  [/checked \d+\/\d+ grasps/i, 'Checking grip options…'],
  [
    /Best grasp: (outside|inside) grip.*?approach (\d+(?:\.\d+)?) ?deg/i,
    (m) => `Best grip found: from ${m[1].toLowerCase()}, ${approach(Number(m[2]))}`,
    'ok'
  ],
  [/inside a cavity/i, "One grip point is inside a hole, that's OK", 'info'],
  [/touch motion\(s\) planned/i, 'Path to the point found', 'ok'],
  [/^Planned|plan(?:ning)? (?:done|finished)|grasps? found/i, 'Grip found', 'ok'],
  [/no (?:reachable|complete|collision-free).*(?:grasp|motion|path)|cannot reach|can't reach/i,
    "The robot can't reach this grip. Try moving the grip points or the part position.", 'error'],
  [/surfaces face the same way/i, 'Those two spots face the same way. Pick spots on opposite sides.', 'error'],
  [/would slide off/i, 'Those two spots are not opposite enough. The fingers would slip.', 'error'],
  [/opens only \d+ mm/i, 'The part is too wide there for the gripper.', 'error'],
  [/points are in the same place/i, 'Both spots are in the same place. Pick one spot for each finger.', 'error'],
  [/Mark two points on the part/i, 'Click two spots on the part first.', 'warn'],
  [/Mark the point the fingertips/i, 'Click one spot on the part first.', 'warn'],
  [/Load a CAD file first/i, 'Load a part first.', 'warn'],
  [/needs a STEP/i, 'This file type is not supported. Use a STEP file.', 'error'],
  [/gripper is still loading/i, 'The gripper is still loading. Try again in a moment.', 'warn'],
  [/was (?:moved|rotated) after planning|offset changed after planning/i,
    'The grip was changed. Press "Find best grip" again.', 'warn'],
  // ---- simulation
  [/Building (?:the )?MuJoCo scene/i, 'Preparing simulation…'],
  [/Starting the simulation/i, 'Preparing simulation…'],
  [/Putting the arm at the pick point/i, 'Moving to the part…'],
  [/arm (?:is )?at the pick point/i, 'The gripper is at the grip points', 'ok'],
  [/^Converting .* to meshes/i, 'Preparing the part model…'],
  [/Letting the part settle/i, 'Preparing simulation…'],
  [/Analysing the part geometry|ways to reach both points|searching the whole part/i, 'Checking grip options…'],
  [/part (?:landed|is) in the (?:tray|bin)|placed in the (?:tray|bin)/i, 'The part was placed in the bin', 'ok'],
  [/missed the (?:tray|bin)|not in the (?:tray|bin)/i, 'The part did not land in the bin. Check the grip.', 'warn'],
  [/slipped|dropped/i, 'The part slipped in the gripper. Try other grip points.', 'warn'],
  [/Real Sim is now enabled/i, 'Simulation finished. You can now run it on the robot.', 'ok'],
  // ---- robot
  [/TEACH mode/i, 'The robot is in teach mode. Switch the pendant to REMOTE.', 'error'],
  [/Connecting to the robot/i, 'Connecting to the robot…'],
  [/Starting Real Sim/i, 'Starting the robot…'],
  [/could not connect|connection refused|no route to host|timed out|unreachable|lost contact/i,
    'Robot not reachable. Check the network cable and the robot address in Settings.', 'error'],
  [/gripper did not finish|ttyUSB|serial port|modbus/i, 'Gripper not found. Check the USB cable.', 'error'],
  [/a joint would turn/i, 'The robot is too far from its start position. Press "Send robot home" first.', 'error'],
  [/stopped \d+(?:\.\d+)? ?deg away/i, 'The robot stopped before the target. Check that nothing blocks it.', 'error'],
  [/alarm/i, 'The robot has an alarm. Clear it on the pendant.', 'error'],
  [/Go Home/i, 'Sending the robot home…'],
  [/home pose reached|at home|reached home/i, 'The robot is at home', 'ok'],
  [/Stop sent/i, 'Stop sent to the robot', 'warn'],
  [/Could not send stop/i, 'Stop could not be sent. Use the emergency stop.', 'error'],
  [/cell server/i, 'The app lost its connection. Restart Pick Cell.', 'error']
];

const approach = (deg: number) => (deg < 5 ? 'straight down' : `tilted ${Math.round(deg)}°`);

/** The operator text for a raw message, or the unknown-error text for an unmatched error. */
export function friendly(raw: string, kind: 'info' | 'ok' | 'error' | 'warn' = 'info'): Friendly {
  const text = (raw || '').trim();
  for (const [re, out, k] of RULES) {
    const m = text.match(re);
    if (m) return { text: typeof out === 'string' ? out : out(m), kind: k ?? kind };
  }
  if (kind === 'error') return { text: UNKNOWN_ERROR, kind: 'error' };
  // Unmatched progress text: shown only if it is already plain (short, no jargon).
  if (text.length <= 90 && !JARGON.test(text)) return { text, kind };
  return { text: kind === 'ok' ? 'Done' : 'Working…', kind };
}

export type Job = 'plan' | 'approach' | 'sim' | 'real' | 'home' | 'stop' | 'ui' | 'server' | null;

/** The operator text for one event of a job: knows what a finished or failed job means. */
export function friendlyEvent(job: Job, phase: 'start' | 'update' | 'ok' | 'error', raw: string): Friendly {
  if (phase === 'ok') {
    const f = friendly(raw, 'ok');
    if (job === 'plan') return f.kind === 'ok' && f.text !== 'Done' ? f : { text: 'Grip found', kind: 'ok' };
    if (job === 'approach') return { text: 'The gripper is at the grip points', kind: 'ok' };
    if (job === 'sim') return /did not land|slipped/.test(f.text) ? f : { text: 'Simulation finished. You can now run it on the robot.', kind: 'ok' };
    if (job === 'real') return { text: 'Robot run finished', kind: 'ok' };
    if (job === 'home') return { text: 'The robot is at home', kind: 'ok' };
    return f;
  }
  if (phase === 'error') {
    const f = friendly(raw, 'error');
    if (job === 'plan' && f.text === UNKNOWN_ERROR) {
      return { text: "The robot can't reach this grip. Try moving the grip points or the part position.", kind: 'error' };
    }
    return f;
  }
  return friendly(raw, 'info');
}

/** Words that must not reach the operator screens. */
export const JARGON = /grasp|approach|\bIK\b|MuJoCo|TCP|joint|candidate|waypoint|quaternion|RRT|collision|var\/runs|Real Sim/i;

/** Progress shown on the run step, from the job's phase (or its message when there is no phase). */
export function progressLabel(phase: string | undefined, message: string | undefined): string | null {
  const p = (phase || '').toLowerCase();
  if (/^(pre|approach|grip|hold|lift|close)/.test(p)) return 'Picking…';
  if (/^(to-camera|inspect|look)/.test(p)) return 'Showing the part to the camera…';
  if (/^(transfer|place|release)/.test(p)) return 'Placing…';
  if (/^(retreat|home)/.test(p)) return 'Going home…';
  const m = (message || '').toLowerCase();
  if (/pick|grip|clos|lift/.test(m)) return 'Picking…';
  if (/camera|inspect/.test(m)) return 'Showing the part to the camera…';
  if (/place|release|tray|bin/.test(m)) return 'Placing…';
  if (/home/.test(m)) return 'Going home…';
  return null;
}

/** Header pill state for the robot and the gripper, worked out from messages (there is no live status feed). */
export type PillState = 'ok' | 'warn' | 'error' | 'idle';
export interface Pill {
  state: PillState;
  text: string;
}

export interface Pills {
  robot: Pill;
  gripper: Pill;
}

export const INITIAL_PILLS: Pills = {
  robot: { state: 'idle', text: 'Robot not checked yet' },
  gripper: { state: 'idle', text: 'Gripper not checked yet' }
};

/** Updates the pills from one event of a robot job (job = 'real' | 'home'; others leave the pills alone). */
export function nextPills(p: Pills, job: string | null, phase: 'start' | 'update' | 'ok' | 'error', raw: string, realGripper: boolean): Pills {
  const out: Pills = { robot: { ...p.robot }, gripper: { ...p.gripper } };
  if (job === 'server') {
    out.robot = phase === 'error'
      ? { state: 'error', text: 'App not running. Restart Pick Cell.' }
      : p.robot.text.startsWith('App not running') ? INITIAL_PILLS.robot : p.robot;
    return out;
  }
  if (job !== 'real' && job !== 'home') return p;
  if (!realGripper && job === 'real') out.gripper = { state: 'idle', text: 'Gripper: simulation only' };
  if (phase === 'start') {
    out.robot = { state: 'warn', text: 'Connecting to robot…' };
    if (realGripper && job === 'real') out.gripper = { state: 'warn', text: 'Connecting to gripper…' };
    return out;
  }
  const f = friendly(raw, phase === 'error' ? 'error' : 'info');
  if (phase === 'error') {
    if (/Gripper not found/.test(f.text)) out.gripper = { state: 'error', text: f.text };
    else if (/Robot not reachable|teach mode|alarm|App lost/.test(f.text)) out.robot = { state: 'error', text: f.text };
    else out.robot = { state: 'error', text: f.text === UNKNOWN_ERROR ? 'Robot stopped with a problem. See What happened.' : f.text };
    return out;
  }
  // running or finished: the robot answered
  out.robot = { state: 'ok', text: 'Robot connected' };
  if (realGripper && job === 'real' && (phase === 'ok' || /gripper|grip|clos/i.test(raw))) out.gripper = { state: 'ok', text: 'Gripper ready' };
  return out;
}
