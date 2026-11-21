/**
 * The operator's four steps (Load part -> Teach grip -> Simulate -> Run on robot) and what unlocks each one.
 * Pure functions, no DOM: tested by tests/steps.test.mjs.
 */
export type StepId = 1 | 2 | 3 | 4;

export const STEPS: { id: StepId; label: string }[] = [
  { id: 1, label: 'Load part' },
  { id: 2, label: 'Teach grip' },
  { id: 3, label: 'Simulate' },
  { id: 4, label: 'Run on robot' }
];

/** What has been done so far (from the controller's state). */
export interface StepFacts {
  partLoaded: boolean;
  planned: boolean; // a grip was found for the marked points
  simChecked: boolean; // a full simulation of that grip ran to the end
  robotDone: boolean; // the last run on the robot finished well
}

export function stepDone(id: StepId, f: StepFacts): boolean {
  switch (id) {
    case 1:
      return f.partLoaded;
    case 2:
      return f.partLoaded && f.planned;
    case 3:
      return f.partLoaded && f.planned && f.simChecked;
    case 4:
      return f.partLoaded && f.planned && f.simChecked && f.robotDone;
  }
}

const NEEDS: Record<Exclude<StepId, 1>, string> = {
  2: 'Load a part first.',
  3: 'Find a grip first: mark the points and press "Find best grip".',
  4: 'Run the simulation to the end first.'
};

/** A step can be opened when every step before it is done. `reason` says what is missing. */
export function canOpen(id: StepId, f: StepFacts): { ok: boolean; reason: string } {
  for (let k = 1; k < id; k++) {
    if (!stepDone(k as StepId, f)) return { ok: false, reason: NEEDS[(k + 1) as Exclude<StepId, 1>] };
  }
  return { ok: true, reason: '' };
}

/** The furthest step that can be opened. */
export function furthestOpen(f: StepFacts): StepId {
  let best: StepId = 1;
  for (const s of STEPS) if (canOpen(s.id, f).ok) best = s.id;
  return best;
}

/** Keeps the shown step valid when what was done changes (e.g. new points drop the old grip). */
export function clampStep(current: StepId, f: StepFacts): StepId {
  return canOpen(current, f).ok ? current : furthestOpen(f);
}

export interface SafetyChecks {
  areaClear: boolean;
  partInPlace: boolean;
}

/** "Start robot" needs a finished simulation, both safety ticks, and nothing else running. */
export function canStartRobot(f: StepFacts, checks: SafetyChecks, busy: boolean): { ok: boolean; reason: string } {
  if (!f.planned) return { ok: false, reason: NEEDS[3] };
  if (!f.simChecked) return { ok: false, reason: NEEDS[4] };
  if (!checks.areaClear && !checks.partInPlace) return { ok: false, reason: 'Tick both safety checks first.' };
  if (!checks.areaClear) return { ok: false, reason: 'Tick "Area is clear" first.' };
  if (!checks.partInPlace) return { ok: false, reason: 'Tick "Part is in place" first.' };
  if (busy) return { ok: false, reason: 'Wait until the current job has finished.' };
  return { ok: true, reason: '' };
}

/** Teach mode: the single source of truth for whether a click on the part may place grip points. */
export function isTeachMode(step: StepId): boolean {
  return step === 2;
}

/** A click on the part places a point only in teach mode, and not while a job or playback runs. */
export function acceptsPointClick(teachMode: boolean, s: { busy: boolean; playing: boolean; liveView: boolean }): boolean {
  return teachMode && !s.busy && !s.playing && !s.liveView;
}

/** The banner over the 3D view in teach mode. */
export function teachBanner(mode: 'grip' | 'touch', placed: number): string {
  if (mode === 'touch') return placed ? 'Teach mode: point placed' : 'Teach mode: click to place the point';
  if (placed >= 2) return 'Teach mode: both points placed';
  return `Teach mode: click to place finger ${placed + 1}`;
}
