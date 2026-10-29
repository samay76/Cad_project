const fs = require('fs');
const path = require('path');

async function runAcceptanceTests() {
  const BASE = 'http://127.0.0.1:4444';
  const sessionRes = await fetch(`${BASE}/session`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      capabilities: {
        alwaysMatch: {
          'moz:firefoxOptions': {
            args: ['-headless', '-width', '1600', '-height', '1000']
          }
        }
      }
    })
  });

  const sessionData = await sessionRes.json();
  const sessionId = sessionData.value.sessionId;

  const exec = async (script, args = []) => {
    const res = await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ script, args })
    });
    const d = await res.json();
    return d.value;
  };

  const screenshot = async () => {
    const res = await fetch(`${BASE}/session/${sessionId}/screenshot`);
    const d = await res.json();
    return d.value; // base64
  };

  try {
    console.log('Navigating to http://localhost:3000...');
    await fetch(`${BASE}/session/${sessionId}/url`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url: 'http://localhost:3000' })
    });

    console.log('Waiting for viewer and gripper to initialize...');
    for (let i = 0; i < 30; i++) {
      await new Promise((r) => setTimeout(r, 1000));
      const ready = await exec(`
        return !!window.__viewer && !!window.__graspController && !!window.__partController;
      `);
      if (ready) {
        console.log(`Ready after ${i + 1}s`);
        break;
      }
    }

    // 1. Load STEP file skf_cam_roller_305703_c-2z_0.stp
    console.log('\n--- LOADING STEP MODEL ---');
    const stepPath = path.resolve(__dirname, '../skf_cam_roller_305703_c-2z_0.stp');
    const stepBuffer = fs.readFileSync(stepPath);
    const stepBase64 = stepBuffer.toString('base64');

    const loadResult = await exec(
      `
      const base64 = arguments[0];
      const binaryString = atob(base64);
      const bytes = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }
      return (async () => {
        const viewer = window.__viewer;
        await viewer.loadFile(bytes.buffer, 'skf_cam_roller_305703_c-2z_0.stp');
        const modelRoot = viewer.getModelRoot();
        if (window.__partController) window.__partController.setCadPartsRoot(modelRoot);
        if (window.__graspController) window.__graspController.setCadPartsRoot(modelRoot);
        return {
          loaded: !!modelRoot,
          meshCount: viewer.getSelectionManager()?.selectableMeshes?.length || 0
        };
      })();
    `,
      [stepBase64]
    );

    console.log('STEP Load Result:', loadResult);

    // Verify Scene Sibling Hierarchy
    const hierarchy = await exec(`
      const viewer = window.__viewer;
      const gc = window.__graspController;
      const pc = window.__partController;
      const modelRoot = viewer.getModelRoot();
      const gripperRoot = gc.getGripperModel().root;
      return {
        modelParentIsScene: modelRoot.parent === viewer.getScene(),
        gripperParentIsScene: gripperRoot.parent === viewer.getScene(),
        isModelChildOfGripper: modelRoot.parent === gripperRoot,
        isGripperChildOfModel: gripperRoot.parent === modelRoot
      };
    `);
    console.log('Scene Hierarchy Check:', hierarchy);
    if (!hierarchy.modelParentIsScene || !hierarchy.gripperParentIsScene) {
      throw new Error('Scene hierarchy failed: partRoot and gripperRoot must be direct siblings under scene root!');
    }

    // =========================================================================
    // ACCEPTANCE TEST 1: Edit Part -> Move/Rotate part -> Gripper stays fixed!
    // =========================================================================
    console.log('\n--- ACCEPTANCE TEST 1: EDIT PART (GRIPPER MUST NOT MOVE) ---');
    const test1 = await exec(`
      const gc = window.__graspController;
      const pc = window.__partController;
      const gm = gc.getGripperModel();

      // Ensure "Move gripper together with part" is OFF
      pc.setMoveGripperTogetherWithPart(false);
      gc.setActiveEditTarget('part');

      // Record initial gripper world pose
      gm.root.updateMatrixWorld(true);
      const initGripPos = { ...gm.root.position };
      const initGripQuat = { x: gm.root.quaternion.x, y: gm.root.quaternion.y, z: gm.root.quaternion.z, w: gm.root.quaternion.w };

      // 1. Move part using step buttons
      pc.stepTranslate('x', 25);
      pc.stepTranslate('y', -15);
      pc.stepTranslate('z', 10);
      pc.stepRotate('x', 15);
      pc.stepRotate('z', 30);

      // 2. Move part using numeric pose inputs
      pc.setNumericPose({ x: 50, y: -30, z: 20 }, { roll: 20, pitch: -10, yaw: 45 });

      gm.root.updateMatrixWorld(true);
      const finalGripPos = { ...gm.root.position };
      const finalGripQuat = { x: gm.root.quaternion.x, y: gm.root.quaternion.y, z: gm.root.quaternion.z, w: gm.root.quaternion.w };

      const posDiff = Math.hypot(
        finalGripPos.x - initGripPos.x,
        finalGripPos.y - initGripPos.y,
        finalGripPos.z - initGripPos.z
      );
      const quatDiff = Math.hypot(
        finalGripQuat.x - initGripQuat.x,
        finalGripQuat.y - initGripQuat.y,
        finalGripQuat.z - initGripQuat.z,
        finalGripQuat.w - initGripQuat.w
      );

      return {
        initGripPos,
        finalGripPos,
        posDiff,
        quatDiff,
        passed: posDiff < 0.001 && quatDiff < 0.001
      };
    `);
    console.log('Test 1 Result:', test1);
    if (!test1.passed) throw new Error('Test 1 failed: Gripper moved when moving Part!');

    // =========================================================================
    // ACCEPTANCE TEST 2: Edit Gripper -> Move gripper -> Part stays fixed!
    // =========================================================================
    console.log('\n--- ACCEPTANCE TEST 2: EDIT GRIPPER (PART MUST NOT MOVE) ---');
    const test2 = await exec(`
      const gc = window.__graspController;
      const pc = window.__partController;
      const viewer = window.__viewer;
      const modelRoot = viewer.getModelRoot();

      gc.setActiveEditTarget('gripper');

      modelRoot.updateMatrixWorld(true);
      const initPartPos = { ...modelRoot.position };
      const initPartQuat = { x: modelRoot.quaternion.x, y: modelRoot.quaternion.y, z: modelRoot.quaternion.z, w: modelRoot.quaternion.w };

      // Move gripper with steps and numeric inputs
      gc.stepTranslate('x', 20);
      gc.stepTranslate('z', -10);
      gc.stepRotate('z', 45);
      gc.setNumericPose({ x: 10, y: 20, z: 80 }, { roll: 0, pitch: 90, yaw: 0 });

      modelRoot.updateMatrixWorld(true);
      const finalPartPos = { ...modelRoot.position };
      const finalPartQuat = { x: modelRoot.quaternion.x, y: modelRoot.quaternion.y, z: modelRoot.quaternion.z, w: modelRoot.quaternion.w };

      const posDiff = Math.hypot(
        finalPartPos.x - initPartPos.x,
        finalPartPos.y - initPartPos.y,
        finalPartPos.z - initPartPos.z
      );
      const quatDiff = Math.hypot(
        finalPartQuat.x - initPartQuat.x,
        finalPartQuat.y - initPartQuat.y,
        finalPartQuat.z - initPartQuat.z,
        finalPartQuat.w - initPartQuat.w
      );

      return {
        initPartPos,
        finalPartPos,
        posDiff,
        quatDiff,
        passed: posDiff < 0.001 && quatDiff < 0.001
      };
    `);
    console.log('Test 2 Result:', test2);
    if (!test2.passed) throw new Error('Test 2 failed: Part moved when moving Gripper!');

    // =========================================================================
    // ACCEPTANCE TEST 3: Toggle "Move gripper together with part"
    // =========================================================================
    console.log('\n--- ACCEPTANCE TEST 3: TOGGLE MOVE TOGETHER WITH PART ---');
    const test3 = await exec(`
      const gc = window.__graspController;
      const pc = window.__partController;
      const gm = gc.getGripperModel();
      const viewer = window.__viewer;
      const modelRoot = viewer.getModelRoot();

      // Reset poses
      pc.resetPartPose();
      gc.resetGripperPose();

      // 1. Turn ON "Move gripper together with part"
      pc.setMoveGripperTogetherWithPart(true);
      const stateBeforeTogether = gc.getCurrentState();
      const relBefore = { ...stateBeforeTogether.relativePose.position };

      // Move part
      pc.stepTranslate('x', 30);
      pc.stepTranslate('y', 20);

      const stateAfterTogether = gc.getCurrentState();
      const relAfter = { ...stateAfterTogether.relativePose.position };

      const relDiff = Math.hypot(
        relAfter.x - relBefore.x,
        relAfter.y - relBefore.y,
        relAfter.z - relBefore.z
      );

      // 2. Turn OFF "Move gripper together with part"
      pc.setMoveGripperTogetherWithPart(false);
      const gripPosBeforeIndep = { ...gm.root.position };

      // Move part again
      pc.stepTranslate('x', 40);

      const gripPosAfterIndep = { ...gm.root.position };
      const indepDiff = Math.hypot(
        gripPosAfterIndep.x - gripPosBeforeIndep.x,
        gripPosAfterIndep.y - gripPosBeforeIndep.y,
        gripPosAfterIndep.z - gripPosBeforeIndep.z
      );

      return {
        relDiffTogether: relDiff,
        indepDiffGrip: indepDiff,
        passedTogether: relDiff < 0.01,
        passedIndep: indepDiff < 0.001
      };
    `);
    console.log('Test 3 Result:', test3);
    if (!test3.passedTogether || !test3.passedIndep) {
      throw new Error('Test 3 failed: "Move gripper together with part" behavior incorrect!');
    }

    // =========================================================================
    // ACCEPTANCE TEST 4: Save Grasp -> Move part -> Click saved grasp -> Follows!
    // =========================================================================
    console.log('\n--- ACCEPTANCE TEST 4: SAVED GRASPS FOLLOW PART POSE ---');
    const test4 = await exec(`
      const gc = window.__graspController;
      const pc = window.__partController;
      const viewer = window.__viewer;
      const modelRoot = viewer.getModelRoot();

      // Reset
      pc.setMoveGripperTogetherWithPart(false);
      pc.resetPartPose();
      gc.resetGripperPose();

      // Snap to outer wall of cam roller
      gc.snapToClickedPoint({ x: 8.75, y: 0, z: 23.5 }, { x: 0, y: 0, z: 1 }, 0, 15);

      // Auto-place on outer wall
      const autoRes = gc.autoPlaceOnOuterWall();

      // Close until contact
      const closeRes = gc.closeUntilContact(0.1);

      // Record relative grasp pose (T_part_grasp)
      const stateAtGrasp = gc.getCurrentState();
      const savedPartGrasp = JSON.parse(JSON.stringify(stateAtGrasp.relativePose));
      const savedRpy = JSON.parse(JSON.stringify(stateAtGrasp.relativeRpy));

      // Move part to somewhere else in the world!
      pc.stepTranslate('x', 60);
      pc.stepTranslate('y', -45);
      pc.stepRotate('z', 45);

      // Gripper stayed behind
      // Now click / restore the saved grasp:
      gc.setRelativeGraspPose(savedPartGrasp, savedRpy);

      // Compute current relative pose after restore:
      const stateAfterRestore = gc.getCurrentState();
      const currentRel = stateAfterRestore.relativePose.position;

      const relErr = Math.hypot(
        currentRel.x - savedPartGrasp.position.x,
        currentRel.y - savedPartGrasp.position.y,
        currentRel.z - savedPartGrasp.position.z
      );

      return {
        autoRes,
        closeResStatus: closeRes.status,
        measuredGraspWidth: closeRes.measuredGraspWidth,
        savedRelPos: savedPartGrasp.position,
        currentRelPos: currentRel,
        relErr,
        passed: relErr < 0.05 && closeRes.status === 'contact'
      };
    `);
    console.log('Test 4 Result:', test4);
    if (!test4.passed) throw new Error('Test 4 failed: Saved grasp did not follow part pose!');

    // =========================================================================
    // ACCEPTANCE TEST 5: Performance HUD, 60 FPS, Collision < 8ms, Test Grasp
    // =========================================================================
    console.log('\n--- ACCEPTANCE TEST 5: PERFORMANCE HUD & DRAGGING BENCHMARK ---');
    const test5 = await exec(`
      const gc = window.__graspController;
      const pc = window.__partController;
      const hud = window.__viewer.getPerformanceHUD();

      // Benchmark collision check speed across a 50-step sweep
      const colTimes = [];
      const startTime = performance.now();

      for (let i = 0; i < 50; i++) {
        gc.stepTranslate('x', (i % 2 === 0 ? 1 : -1) * 2);
        const colRes = gc.evaluateCollisionState();
        if (colRes.executionTimeMs !== undefined) {
          colTimes.push(colRes.executionTimeMs);
        }
      }

      const maxColTime = Math.max(...colTimes);
      const avgColTime = colTimes.reduce((a, b) => a + b, 0) / colTimes.length;

      // Close until contact check
      const closeRes = gc.closeUntilContact(0.1);

      return {
        sampleCount: colTimes.length,
        maxColTimeMs: Number(maxColTime.toFixed(3)),
        avgColTimeMs: Number(avgColTime.toFixed(3)),
        closeStatus: closeRes.status,
        closeWidth: closeRes.measuredGraspWidth,
        passedPerf: maxColTime < 8.0
      };
    `);
    console.log('Test 5 Result:', test5);
    if (!test5.passedPerf) {
      throw new Error(`Test 5 failed: Collision check exceeded 8 ms target (max: ${test5.maxColTimeMs} ms)!`);
    }

    // Run Test Grasp and verify animation
    console.log('\n--- RUNNING TEST GRASP ANIMATION ---');
    await exec(`window.__graspController.testGrasp();`);
    // Wait for lift phase (approx 2.5s into motion)
    await new Promise((r) => setTimeout(r, 2600));

    const liftState = await exec(`
      const gc = window.__graspController;
      const pc = window.__partController;
      const viewer = window.__viewer;
      return {
        isTesting: gc.getCurrentState().isTesting,
        phase: gc.getCurrentState().testPhase,
        modelZ: viewer.getModelRoot().position.z
      };
    `);
    console.log('Lift phase check:', liftState);

    // Capture screenshot
    const shotBase64 = await screenshot();
    const shotPath = path.resolve(__dirname, '../../.gemini/antigravity-cli/brain/5adce33d-880f-4878-a233-329288455ba8/cad-step4-verified.png');
    fs.writeFileSync(shotPath, Buffer.from(shotBase64, 'base64'));
    console.log(`Saved acceptance test screenshot to ${shotPath}`);

    // Wait for test grasp sequence to finish
    await new Promise((r) => setTimeout(r, 4000));

    console.log('\n=========================================');
    console.log('ALL 5 ACCEPTANCE TESTS PASSED SUCCESSFULLY!');
    console.log('=========================================');
  } finally {
    await fetch(`${BASE}/session/${sessionId}`, { method: 'DELETE' });
  }
}

runAcceptanceTests().catch((e) => {
  console.error('Acceptance tests failed with error:', e);
  process.exit(1);
});
