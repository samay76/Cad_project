// Step gating and plain-language mapping (src/ui/steps.ts, src/ui/friendly.ts). Run: npm test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';

const load = async (entry) => {
  const out = await build({ entryPoints: [entry], bundle: true, format: 'esm', write: false, platform: 'neutral', logLevel: 'silent' });
  return import('data:text/javascript;base64,' + Buffer.from(out.outputFiles[0].text).toString('base64'));
};
const steps = await load(new URL('../src/ui/steps.ts', import.meta.url).pathname);
const fr = await load(new URL('../src/ui/friendly.ts', import.meta.url).pathname);

const facts = (o = {}) => ({ partLoaded: false, planned: false, simChecked: false, robotDone: false, ...o });
const both = { areaClear: true, partInPlace: true };

test('only step 1 opens before a part is loaded', () => {
  const f = facts();
  assert.equal(steps.canOpen(1, f).ok, true);
  for (const s of [2, 3, 4]) assert.equal(steps.canOpen(s, f).ok, false);
  assert.match(steps.canOpen(2, f).reason, /Load a part/);
  assert.match(steps.canOpen(4, f).reason, /Load a part/); // the first missing step is named
});

test('each step needs the one before it', () => {
  assert.equal(steps.canOpen(2, facts({ partLoaded: true })).ok, true);
  assert.equal(steps.canOpen(3, facts({ partLoaded: true })).ok, false);
  assert.match(steps.canOpen(3, facts({ partLoaded: true })).reason, /Find best grip/);
  assert.equal(steps.canOpen(3, facts({ partLoaded: true, planned: true })).ok, true);
  assert.equal(steps.canOpen(4, facts({ partLoaded: true, planned: true })).ok, false);
  assert.match(steps.canOpen(4, facts({ partLoaded: true, planned: true })).reason, /simulation/);
  assert.equal(steps.canOpen(4, facts({ partLoaded: true, planned: true, simChecked: true })).ok, true);
});

test('completed steps can always be reopened', () => {
  const f = facts({ partLoaded: true, planned: true, simChecked: true });
  for (const s of [1, 2, 3, 4]) assert.equal(steps.canOpen(s, f).ok, true);
});

test('a dropped grip pulls the screen back to a step that is still valid', () => {
  assert.equal(steps.clampStep(4, facts({ partLoaded: true })), 2);
  assert.equal(steps.clampStep(3, facts()), 1);
  assert.equal(steps.clampStep(2, facts({ partLoaded: true, planned: true })), 2);
});

test('Start robot needs a finished simulation and both safety ticks', () => {
  const ready = facts({ partLoaded: true, planned: true, simChecked: true });
  assert.equal(steps.canStartRobot(ready, both, false).ok, true);
  assert.equal(steps.canStartRobot(facts({ partLoaded: true, planned: true }), both, false).ok, false);
  assert.match(steps.canStartRobot(ready, { areaClear: false, partInPlace: true }, false).reason, /Area is clear/);
  assert.match(steps.canStartRobot(ready, { areaClear: true, partInPlace: false }, false).reason, /Part is in place/);
  assert.match(steps.canStartRobot(ready, { areaClear: false, partInPlace: false }, false).reason, /both/);
  assert.equal(steps.canStartRobot(ready, both, true).ok, false); // something else is running
});

test('teach mode is on only in the Teach grip step', () => {
  assert.deepEqual([1, 2, 3, 4].map(steps.isTeachMode), [false, true, false, false]);
});

test('a click on the model outside Teach grip does not change the grip points', () => {
  const idle = { busy: false, playing: false, liveView: false };
  for (const step of [1, 3, 4]) assert.equal(steps.acceptsPointClick(steps.isTeachMode(step), idle), false);
  assert.equal(steps.acceptsPointClick(steps.isTeachMode(2), idle), true);
  // even in Teach grip, not while something runs
  assert.equal(steps.acceptsPointClick(true, { ...idle, busy: true }), false);
  assert.equal(steps.acceptsPointClick(true, { ...idle, liveView: true }), false);
});

test('teach banner names the next finger', () => {
  assert.equal(steps.teachBanner('grip', 0), 'Teach mode: click to place finger 1');
  assert.equal(steps.teachBanner('grip', 1), 'Teach mode: click to place finger 2');
  assert.equal(steps.teachBanner('grip', 2), 'Teach mode: both points placed');
});

test('backend messages become plain language', () => {
  assert.equal(fr.friendly('Found 3/4 complete motions (checked 12/40 grasps)').text, 'Checking grip options…');
  assert.equal(fr.friendly('Building MuJoCo scene (EC66 + 2F-85 + part)...').text, 'Preparing simulation…');
  assert.equal(fr.friendly('Putting the arm at the pick point...').text, 'Moving to the part…');
  assert.equal(
    fr.friendly('3 complete motion(s) planned in 12 s. Best grasp: outside grip, 6.0 mm wide, approach 0 deg from vertical.', 'ok').text,
    'Best grip found: from outside, straight down'
  );
  assert.equal(fr.friendly('One point is inside a cavity, so the fingers open there.').text, "One grip point is inside a hole, that's OK");
  assert.equal(fr.friendly('REAL robot: Lowering the part (smooth, 5 points, 3% speed)').text, 'Lowering the part…');
  assert.equal(fr.friendly('KeyError: candidate 7', 'error').text, fr.UNKNOWN_ERROR);
  assert.equal(fr.friendlyEvent('plan', 'error', 'Main reasons: wrist hits the table.').text.startsWith("The robot can't reach"), true);
});

test('no operator text uses engineering words', () => {
  const samples = [
    'Found 3/4 complete motions (checked 12/40 grasps)',
    'Best grasp: inside grip, 20.0 mm wide, approach 30 deg from vertical.',
    'Building MuJoCo scene...',
    'Real move refused: a joint would turn 140 deg',
    'Robot run: waypoint 4 of 9 (TCP 3 mm from target)'
  ];
  for (const raw of samples) for (const kind of ['info', 'ok', 'error']) {
    const t = fr.friendly(raw, kind).text;
    assert.doesNotMatch(t, fr.JARGON, `"${raw}" -> "${t}"`);
    assert.doesNotMatch(t, /!|successfully/i);
  }
});

test('status pills: robot errors show the fix', () => {
  let p = fr.INITIAL_PILLS;
  p = fr.nextPills(p, 'real', 'start', '', true);
  assert.equal(p.robot.state, 'warn');
  p = fr.nextPills(p, 'real', 'error', 'Could not connect to 192.168.1.200: timed out', true);
  assert.equal(p.robot.state, 'error');
  assert.match(p.robot.text, /network cable/);
  p = fr.nextPills(fr.INITIAL_PILLS, 'real', 'error', 'The gripper did not finish closing (/dev/ttyUSB0)', true);
  assert.equal(p.gripper.state, 'error');
  assert.equal(p.gripper.text, 'Gripper not found. Check the USB cable.');
  p = fr.nextPills(fr.INITIAL_PILLS, 'real', 'ok', 'done', true);
  assert.deepEqual([p.robot.state, p.gripper.state], ['ok', 'ok']);
  assert.equal(fr.nextPills(fr.INITIAL_PILLS, 'sim', 'error', 'x', true), fr.INITIAL_PILLS); // sim does not touch them
});
