import * as THREE from 'three';
import { AppLayout } from './ui/Layout';
import { Viewer } from './viewer/Viewer';
import { PartsListView } from './ui/PartsList';
import { PropertiesPanel } from './ui/PropertiesPanel';
import { GripperPanel } from './ui/GripperPanel';
import { PartPanel } from './ui/PartPanel';
import { GraspListView } from './ui/GraspList';
import { CameraFeedPanel } from './ui/CameraFeedPanel';
import { GripperLoader } from './gripper/GripperLoader';
import { GraspController } from './gripper/GraspController';
import { PartController } from './part/PartController';
import type { SavedGrasp, ClickedPointInfo, GripperState, TrajectorySample, GripMode, Pose } from './types';
import { GRIPPERS, applyGripperSpec, getStoredGripperId, type GripperId } from './config';
import './styles.css';

document.addEventListener('DOMContentLoaded', async () => {
  const appElement = document.getElementById('app');
  if (!appElement) throw new Error('App root element #app not found.');

  // Pre-declare component instances
  let viewer: Viewer | null = null;
  let layout: AppLayout | null = null;
  let partsList: PartsListView | null = null;
  let propertiesPanel: PropertiesPanel | null = null;
  let gripperPanel: GripperPanel | null = null;
  let partPanel: PartPanel | null = null;
  let graspList: GraspListView | null = null;
  let graspController: GraspController | null = null;
  let partController: PartController | null = null;
  let cameraPanel: CameraFeedPanel | null = null;

  // State
  let lastClickedPoint: ClickedPointInfo | null = null;
  let currentModelName = 'model.step';
  let currentModelBuffer: ArrayBuffer | null = null;
  let savedGrasps: SavedGrasp[] = [];
  let activeGripperId: GripperId = getStoredGripperId();
  applyGripperSpec(GRIPPERS[activeGripperId]);

  // Persistence helpers
  const STORAGE_KEY = 'nferent_cad_saved_grasps';
  const loadGraspsFromStorage = (): SavedGrasp[] => {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (raw) {
        const parsed = JSON.parse(raw);
        if (Array.isArray(parsed)) return parsed;
      }
    } catch {
      // ignore storage errors
    }
    return [];
  };

  const saveGraspsToStorage = () => {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(savedGrasps));
    } catch {
      // ignore storage errors
    }
  };

  savedGrasps = loadGraspsFromStorage();

  // Helper to load CAD geometry safely
  const loadCADBuffer = async (buffer: ArrayBuffer, fileName: string) => {
    if (!viewer || !layout) {
      throw new Error('3D Viewer is still initializing. Please try again in a moment.');
    }
    currentModelName = fileName;
    layout.setLoadedFileName(fileName);
    await viewer.loadFile(buffer, fileName);
    currentModelBuffer = buffer.slice(0);

    const modelRoot = viewer.getModelRoot();
    if (partController) {
      partController.setCadPartsRoot(modelRoot);
    }
    if (graspController) {
      graspController.setCadPartsRoot(modelRoot);
    }
  };

  // Helper to snap gripper to surface
  const snapToLastClicked = () => {
    if (!graspController) {
      layout?.showError('Please load the gripper first.');
      return;
    }
    if (!lastClickedPoint) {
      layout?.showError('Please click a point on the CAD model surface first.');
      return;
    }
    graspController.snapToClickedPoint(
      lastClickedPoint.worldPoint,
      lastClickedPoint.worldNormal
    );
  };

  // Helper to save current grasp pose
  const saveCurrentGrasp = () => {
    if (!graspController) {
      layout?.showError('Please load the gripper before saving a grasp pose.');
      return;
    }
    const state = graspController.getCurrentState();
    const count = savedGrasps.length + 1;
    const newGrasp: SavedGrasp = {
      id: `grasp_${Date.now()}_${Math.random().toString(36).substring(2, 7)}`,
      name: `Grasp Pose ${count}`,
      tcpPose: state.relativePose, // Stored as T_part_grasp relative to CAD part frame
      rpy: state.relativeRpy,
      openingWidth: state.openingWidth,
      measuredGraspWidth: state.measuredGraspWidth,
      contactPoints: graspController.getContactPointsPart(),
      standoff: state.standoff,
      depth: state.depth,
      approachAxis: { x: 0, y: 0, z: 1 },
      closingAxis: { x: 1, y: 0, z: 0 },
      preGraspDistance: state.preGraspDistance,
      tcpOffset: state.tcpOffset,
      createdAt: Date.now(),
      partName: currentModelName,
      gripperId: activeGripperId
    };

    savedGrasps.push(newGrasp);
    saveGraspsToStorage();
    graspList?.setGrasps(savedGrasps);
    graspList?.setSelectedGraspId(newGrasp.id);
  };

  // ---- Motion recording: the gripper path relative to the part, replayed by the MuJoCo simulation ----
  const RECORD_INTERVAL_MS = 50;
  const RECORD_MAX_SAMPLES = 3000;
  let recordTimer: number | null = null;
  let recordStart = 0;
  let recordSamples: TrajectorySample[] = [];

  const poseQuat = (p: Pose) =>
    new THREE.Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  const poseDistMm = (a: Pose, b: Pose) =>
    Math.hypot(a.position.x - b.position.x, a.position.y - b.position.y, a.position.z - b.position.z);
  const poseAngleDeg = (a: Pose, b: Pose) => THREE.MathUtils.radToDeg(poseQuat(a).angleTo(poseQuat(b)));

  const recordSample = () => {
    if (!graspController) return;
    const st = graspController.getCurrentState();
    const pose: Pose = {
      position: { ...st.relativePose.position },
      orientation: { ...st.relativePose.orientation }
    };
    const last = recordSamples[recordSamples.length - 1];
    if (
      last &&
      poseDistMm(last.pose, pose) < 0.2 &&
      poseAngleDeg(last.pose, pose) < 0.2 &&
      Math.abs(last.width - st.openingWidth) < 0.1
    ) {
      return; // nothing moved
    }
    recordSamples.push({ t: Date.now() - recordStart, pose, width: st.openingWidth });
    graspList?.setRecording(true, recordSamples.length);
    if (recordSamples.length >= RECORD_MAX_SAMPLES) {
      layout?.showInfo('Recording', `Reached ${RECORD_MAX_SAMPLES} poses; recording stopped and saved.`);
      stopRecording();
    }
  };

  const startRecording = () => {
    if (!graspController) {
      layout?.showError('Load the gripper before recording a motion.', 'Recording');
      return;
    }
    if (!viewer?.getModelRoot()) {
      layout?.showError('Load the CAD part before recording a motion.', 'Recording');
      return;
    }
    // The robot comes from above: an approach axis (TCP +Z) pointing up means the gripper is upside down.
    const q = graspController.getCurrentState().pose.orientation;
    const approach = new THREE.Vector3(0, 0, 1).applyQuaternion(new THREE.Quaternion(q.x, q.y, q.z, q.w));
    if (approach.z > 0.3) {
      layout?.showError(
        'The gripper points UP (it comes from below the part). Press "Flip Approach (180°)" first so the ' +
          'fingers point down at the part, then press Record again.',
        'Recording'
      );
      return;
    }
    recordSamples = [];
    recordStart = Date.now();
    recordSample(); // starting pose
    recordTimer = window.setInterval(recordSample, RECORD_INTERVAL_MS);
    graspList?.setRecording(true, recordSamples.length);
    layout?.showInfo(
      'Recording gripper motion',
      'Move the gripper to the part the way the robot should (gizmo or step buttons), set the finger ' +
        'opening on the way, and finish at the grasp pose. Last, close the fingers on the part (outside ' +
        'grip) or open them against the inner wall (inside grip). Then press "Stop & save recording".\n\n' +
        'Inside grip of a hole: close the fingers (Close 0mm) BEFORE going down, go down only with the ' +
        'fingertip in the hole (the finger is thick ~8 mm above the TCP), then open the fingers. ' +
        'Inside a hole the web contact/collision badge is approximate (box shapes); MuJoCo checks the exact shape.'
    );
  };

  /** Grip direction = how the fingers move after the gripper stops at the grasp pose. */
  const detectGripMode = (samples: TrajectorySample[]): GripMode | null => {
    const final = samples[samples.length - 1];
    let k = samples.length - 1;
    // The pose can drift slightly while the user adjusts the fingers at the grasp.
    // Keep those near-identical samples in the final segment, otherwise a small
    // TCP wiggle can hide the width change and trigger the fallback prompt.
    while (k > 0 && poseDistMm(samples[k - 1].pose, final.pose) < 2 && poseAngleDeg(samples[k - 1].pose, final.pose) < 2) {
      k--;
    }
    const preWidth = samples[k].width;
    // The opening slider moves in 0.5 mm steps; detect a real slider step.
    if (final.width > preWidth + 0.25) return 'inside';
    if (final.width < preWidth - 0.25) return 'outside';
    return null;
  };

  const stopRecording = () => {
    if (recordTimer !== null) window.clearInterval(recordTimer);
    recordTimer = null;
    recordSample(); // final pose
    graspList?.setRecording(false);
    const samples = recordSamples;
    recordSamples = [];
    if (samples.length < 2) {
      layout?.showError('Nothing was recorded: move the gripper while recording.', 'Recording');
      return;
    }
    let gripMode = detectGripMode(samples);
    if (!gripMode) {
      gripMode = confirm(
        'The recording ends without opening or closing the fingers at the grasp pose.\n\n' +
          'OK = grip by CLOSING the fingers onto the part (outside grip)\n' +
          'Cancel = grip by OPENING the fingers against an inner wall (inside grip)'
      )
        ? 'outside'
        : 'inside';
    }
    const state = graspController!.getCurrentState();
    const last = samples[samples.length - 1];
    const count = savedGrasps.length + 1;
    const newGrasp: SavedGrasp = {
      id: `grasp_${Date.now()}_${Math.random().toString(36).substring(2, 7)}`,
      name: `Recorded Grasp ${count}`,
      tcpPose: last.pose,
      rpy: state.relativeRpy,
      openingWidth: last.width,
      contactPoints: graspController!.getContactPointsPart(),
      standoff: state.standoff,
      depth: state.depth,
      approachAxis: { x: 0, y: 0, z: 1 },
      closingAxis: { x: 1, y: 0, z: 0 },
      preGraspDistance: state.preGraspDistance,
      tcpOffset: state.tcpOffset,
      createdAt: Date.now(),
      partName: currentModelName,
      trajectory: samples,
      gripMode,
      gripperId: activeGripperId
    };
    savedGrasps.push(newGrasp);
    saveGraspsToStorage();
    graspList?.setGrasps(savedGrasps);
    graspList?.setSelectedGraspId(newGrasp.id);
    layout?.showInfo(
      'Recording saved',
      `"${newGrasp.name}": ${samples.length} poses, ${gripMode === 'inside' ? 'inside grip (fingers open to grip)' : 'outside grip (fingers close to grip)'}. ` +
        'Press ▶ to replay it in MuJoCo.',
      6000
    );
  };

  // Helper to restore grasp (places gripper at T_world_part * T_part_grasp)
  const restoreGrasp = async (grasp: SavedGrasp) => {
    if (!graspController) {
      layout?.showError('Please load the gripper first.');
      return;
    }
    const gid = grasp.gripperId ?? '2f85';
    if (gid !== activeGripperId) {
      await selectGripper(gid);
      if (activeGripperId !== gid) return;
    }
    graspController.setRelativeGraspPose(grasp.tcpPose, grasp.rpy);
    graspController.setOpeningWidth(
      grasp.measuredGraspWidth !== undefined ? grasp.measuredGraspWidth : grasp.openingWidth
    );
    graspController.setTcpOffset(grasp.tcpOffset ?? 145);
    graspController.setPreGraspDistance(grasp.preGraspDistance ?? 100);
    if (grasp.standoff !== undefined) graspController.setStandoff(grasp.standoff);
    if (grasp.depth !== undefined) graspController.setDepth(grasp.depth);
  };

  // Gripper selector: switches the gripper model in place (TCP pose kept) and remembers the choice
  let switchingGripper = false;
  const selectGripper = async (id: GripperId) => {
    if (id === activeGripperId || switchingGripper) return;
    if (recordTimer !== null) {
      layout?.showError('Stop the recording before switching the gripper.', 'Gripper');
      return;
    }
    const spec = GRIPPERS[id];
    if (!graspController) {
      activeGripperId = id;
      applyGripperSpec(spec);
      layout?.setActiveGripper(id);
      gripperPanel?.setMaxOpening(spec.maxOpeningWidth);
      return;
    }
    switchingGripper = true;
    try {
      layout?.showLoading(true, `Loading ${spec.label} gripper assets...`);
      const model = await GripperLoader.loadGripper(spec);
      activeGripperId = id;
      applyGripperSpec(spec);
      graspController.replaceGripperModel(model);
      layout?.setActiveGripper(id);
      gripperPanel?.setMaxOpening(spec.maxOpeningWidth);
      layout?.showLoading(false);
      layout?.showInfo(
        'Gripper',
        `${spec.label} selected (opening 0-${spec.maxOpeningWidth} mm, TCP ${spec.defaultTcpOffset} mm from the flange).`,
        4000
      );
    } catch (err: any) {
      layout?.showLoading(false);
      layout?.showError(err?.message || `Failed to load ${spec.label}.`, 'Gripper');
    } finally {
      switchingGripper = false;
    }
  };

  // Helper to export grasps as JSON
  const exportGraspsJSON = () => {
    if (savedGrasps.length === 0) {
      layout?.showError('No grasps to export.');
      return;
    }
    const jsonStr = JSON.stringify(savedGrasps, null, 2);
    const blob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const safeModelName = currentModelName.replace(/[^a-zA-Z0-9_-]/g, '_');
    a.href = url;
    a.download = `grasps_${safeModelName}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  // ---- MuJoCo simulation bridge (served by the Vite plugin in vite.config.ts) ----
  const arrayBufferToBase64 = (buf: ArrayBuffer): string => {
    const bytes = new Uint8Array(buf);
    let binary = '';
    for (let i = 0; i < bytes.length; i += 0x8000) {
      binary += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
    }
    return btoa(binary);
  };

  let simPollTimer: number | null = null;

  const pollSimulation = (runId: string) => {
    if (simPollTimer !== null) window.clearInterval(simPollTimer);
    let lastPhase = '';
    simPollTimer = window.setInterval(async () => {
      try {
        const resp = await fetch(`/api/mujoco/status?id=${encodeURIComponent(runId)}`);
        const st = await resp.json();
        if (st.info?.inspection) cameraPanel?.setImages(st.info.inspection);
        if (st.state === 'running' && st.message) cameraPanel?.setCaption(st.message);
        if (st.phase && st.phase !== lastPhase && st.state === 'running') {
          lastPhase = st.phase;
          layout?.showInfo('MuJoCo simulation', st.message || st.phase);
        }
        if (st.state !== 'running') {
          window.clearInterval(simPollTimer!);
          simPollTimer = null;
          cameraPanel?.finish(st.message || '');
          if (st.ok) {
            layout?.showInfo('MuJoCo simulation finished', st.message || 'Pick & place completed.');
          } else {
            layout?.showError(st.message || 'Simulation failed (see terminal running npm run dev).', 'MuJoCo simulation failed');
          }
        }
      } catch {
        // dev server restarting; keep polling
      }
    }, 1000);
  };

  const simulateGraspInMujoco = async (grasp: SavedGrasp) => {
    if (!currentModelBuffer || !viewer?.getModelRoot()) {
      layout?.showError('Load the CAD part (STEP file) first.', 'MuJoCo simulation');
      return;
    }
    if (!/\.(stp|step)$/i.test(currentModelName)) {
      layout?.showError('MuJoCo simulation currently supports STEP parts only.', 'MuJoCo simulation');
      return;
    }
    if (grasp.partName && grasp.partName !== currentModelName) {
      layout?.showError(
        `This grasp was taught on "${grasp.partName}" but "${currentModelName}" is loaded.`,
        'MuJoCo simulation'
      );
      return;
    }

    // T_world_part as shown in the viewer: MuJoCo places the part on the table in this same orientation.
    const root = viewer.getModelRoot()!;
    root.updateMatrixWorld(true);
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    root.matrixWorld.decompose(pos, quat, new THREE.Vector3());

    layout?.showInfo('MuJoCo simulation', 'Starting simulator (first run converts the STEP mesh, ~30 s)...');
    try {
      const resp = await fetch('/api/mujoco/simulate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          grasp,
          partFileName: currentModelName,
          partFileBase64: arrayBufferToBase64(currentModelBuffer),
          partWorldPose: {
            position: { x: pos.x, y: pos.y, z: pos.z },
            orientation: { x: quat.x, y: quat.y, z: quat.z, w: quat.w }
          }
        })
      });
      const res = await resp.json();
      if (!resp.ok || !res.ok) throw new Error(res.message || `HTTP ${resp.status}`);
      cameraPanel?.start(res.runId);
      pollSimulation(res.runId);
    } catch (err: any) {
      layout?.showError(
        `Could not start MuJoCo: ${err?.message || err}. Is the app running via "npm run dev"?`,
        'MuJoCo simulation failed'
      );
    }
  };

  // Helper to import grasps from JSON
  const importGraspsJSON = (imported: SavedGrasp[]) => {
    if (!Array.isArray(imported) || imported.length === 0) {
      layout?.showError('Imported file does not contain valid grasp poses.');
      return;
    }
    const valid = imported
      .filter((g) => g && g.tcpPose && g.tcpPose.position && g.tcpPose.orientation)
      .map((g) => ({
        ...g,
        standoff: g.standoff ?? 0,
        depth: g.depth ?? 15,
        approachAxis: g.approachAxis ?? { x: 0, y: 0, z: 1 },
        closingAxis: g.closingAxis ?? { x: 1, y: 0, z: 0 },
        tcpOffset: g.tcpOffset ?? 145,
        preGraspDistance: g.preGraspDistance ?? 100
      }));

    if (valid.length === 0) {
      layout?.showError('No valid grasp poses found in imported file.');
      return;
    }
    savedGrasps = [...savedGrasps, ...valid];
    saveGraspsToStorage();
    graspList?.setGrasps(savedGrasps);
    if (valid[0] && graspController) {
      restoreGrasp(valid[0]);
      graspList?.setSelectedGraspId(valid[0].id);
    }
  };

  // 1. Initialize master layout
  layout = new AppLayout(appElement, {
    onFileSelected: async (file: File) => {
      if (!layout) return;
      try {
        layout.showLoading(true, `Reading ${file.name}...`);
        const buffer = await file.arrayBuffer();
        await loadCADBuffer(buffer, file.name);
      } catch (err: any) {
        layout.showLoading(false);
        layout.showError(err?.message || `Failed to read file ${file.name}`);
      }
    },
    onSampleSelected: async (sampleName: string, sampleUrl: string) => {
      if (!layout) return;
      try {
        layout.showLoading(true, `Loading sample ${sampleName}...`);
        const response = await fetch(sampleUrl);
        if (!response.ok) {
          throw new Error(`Failed to fetch ${sampleName}: HTTP ${response.status}`);
        }
        const buffer = await response.arrayBuffer();
        await loadCADBuffer(buffer, sampleName);
      } catch (err: any) {
        layout.showLoading(false);
        layout.showError(err?.message || `Failed to load sample ${sampleName}`);
      }
    },
    onUrlSelected: async (url: string) => {
      if (!layout) return;
      try {
        layout.showLoading(true, `Loading CAD model from ${url}...`);
        const response = await fetch(url);
        if (!response.ok) {
          throw new Error(`Failed to fetch file from "${url}" (HTTP ${response.status})`);
        }
        const buffer = await response.arrayBuffer();
        const fileName = url.split('/').pop()?.split('?')[0] || 'model.step';
        await loadCADBuffer(buffer, fileName);
      } catch (err: any) {
        layout.showLoading(false);
        layout.showError(err?.message || `Failed to fetch file from "${url}". Verify the path or URL.`);
      }
    },
    onLoadGripperClick: async () => {
      await loadGripperModel();
    },
    onGripperSelect: (id) => {
      selectGripper(id);
    },
    onTransformModeToggle: (mode) => {
      graspController?.setTransformMode(mode);
    },
    onTargetToggle: (target) => {
      graspController?.setActiveEditTarget(target);
      layout?.setActiveTarget(target);
      partPanel?.setActiveTarget(target);
    },
    onSnapPointClick: () => {
      snapToLastClicked();
    },
    onFlipApproachClick: () => {
      graspController?.flipApproach();
    },
    onTestGraspClick: () => {
      graspController?.testGrasp();
    },
    onStopTestClick: () => {
      graspController?.stopTestGrasp();
    }
  });

  layout.setActiveGripper(activeGripperId);

  // 2. Initialize 3D Viewer into viewport container
  viewer = new Viewer(layout.viewportContainer);
  cameraPanel = new CameraFeedPanel(layout.viewportContainer);

  // 3. Initialize Part Controller
  partController = new PartController();

  // 4. Initialize CAD Properties Panel in right sidebar
  propertiesPanel = new PropertiesPanel(layout.propertiesPanelContainer);

  // 5. Initialize Parts List in left sidebar
  partsList = new PartsListView(layout.partsListContainer, {
    onResetView: () => {
      viewer?.resetView();
    },
    onFitToModel: () => {
      viewer?.fitToModel();
    },
    onToggleWireframe: () => {
      return viewer?.toggleWireframe() ?? false;
    },
    onViewPreset: (preset) => {
      viewer?.setCameraView(preset);
    },
    onPartSelect: (index) => {
      viewer?.selectPartByIndex(index);
    }
  });

  // 6. Initialize Part Pose Panel in right sidebar
  partPanel = new PartPanel(layout.partPanelContainer, {
    onNumericPoseChange: (pos, rpy) => {
      partController?.setNumericPose(pos, rpy);
      graspController?.evaluateCollisionState();
    },
    onStepTranslate: (axis, delta) => {
      partController?.stepTranslate(axis, delta);
      graspController?.evaluateCollisionState();
    },
    onStepRotate: (axis, delta) => {
      partController?.stepRotate(axis, delta);
      graspController?.evaluateCollisionState();
    },
    onLockGripperToggle: (locked) => {
      partController?.setLockGripperToPart(locked);
    },
    onResetPartPose: () => {
      partController?.resetPartPose();
      graspController?.evaluateCollisionState();
    },
    onBakePartOrigin: () => {
      partController?.bakeCurrentOrientationAsOrigin(
        () => {
          // Warning confirmed: saved grasps invalidated
          savedGrasps = [];
          saveGraspsToStorage();
          graspList?.setGrasps(savedGrasps);
        }
      );
    },
    onTargetToggle: (target) => {
      graspController?.setActiveEditTarget(target);
      layout?.setActiveTarget(target);
      partPanel?.setActiveTarget(target);
    }
  });

  partController.setOnStateChange((state) => {
    partPanel?.setPartState(state);
    viewer?.requestRender();
  });

  partController.setOnGripperRelativeChanged(() => {
    if (graspController) {
      gripperPanel?.setGripperState(graspController.getCurrentState());
    }
    viewer?.requestRender();
  });

  // 7. Initialize Gripper Pose Panel in right sidebar
  gripperPanel = new GripperPanel(layout.gripperPanelContainer, {
    onNumericPoseChange: (pos, rpy) => {
      graspController?.setNumericPose(pos, rpy);
    },
    onStepTranslate: (axis, delta) => {
      graspController?.stepTranslate(axis, delta);
    },
    onStepRotate: (axis, delta) => {
      graspController?.stepRotate(axis, delta);
    },
    onOpeningChange: (width) => {
      graspController?.setOpeningWidth(width);
    },
    onTcpOffsetChange: (offset) => {
      graspController?.setTcpOffset(offset);
    },
    onPreGraspDistanceChange: (dist) => {
      graspController?.setPreGraspDistance(dist);
    },
    onSnapToClickedPoint: () => {
      snapToLastClicked();
    },
    onFlipApproach: () => {
      graspController?.flipApproach();
    },
    onRotateApproach: (deg) => {
      graspController?.rotateAroundApproach(deg);
    },
    onResetPose: () => {
      graspController?.resetGripperPose();
    },
    onTestGrasp: () => {
      graspController?.testGrasp();
    },
    onStopTest: () => {
      graspController?.stopTestGrasp();
    },
    onSaveGrasp: () => {
      saveCurrentGrasp();
    },
    onCloseUntilContact: (tol) => {
      graspController?.closeUntilContact(tol);
    },
    onOpenToClear: () => {
      graspController?.openToClear();
    },
    onAutoPlaceOnOuterWall: () => {
      const clickPt = lastClickedPoint
        ? new THREE.Vector3(
            lastClickedPoint.worldPoint.x,
            lastClickedPoint.worldPoint.y,
            lastClickedPoint.worldPoint.z
          )
        : undefined;
      graspController?.autoPlaceOnOuterWall(clickPt);
    },
    onStandoffChange: (s) => {
      graspController?.setStandoff(s);
    },
    onDepthChange: (d) => {
      graspController?.setDepth(d);
    },
    onToleranceChange: (t) => {
      graspController?.setTolerance(t);
    }
  });

  // 8. Initialize Grasps List in left sidebar
  graspList = new GraspListView(layout.graspListContainer, {
    onRestoreGrasp: (grasp) => {
      restoreGrasp(grasp);
    },
    onRenameGrasp: (id, newName) => {
      const g = savedGrasps.find((item) => item.id === id);
      if (g) {
        g.name = newName;
        saveGraspsToStorage();
        graspList?.setGrasps(savedGrasps);
      }
    },
    onDeleteGrasp: (id) => {
      savedGrasps = savedGrasps.filter((item) => item.id !== id);
      saveGraspsToStorage();
      graspList?.setGrasps(savedGrasps);
    },
    onExportJSON: () => {
      exportGraspsJSON();
    },
    onImportJSON: (imported) => {
      importGraspsJSON(imported);
    },
    onSimulateGrasp: (grasp) => {
      if (recordTimer !== null) {
        layout?.showError('Stop the recording first.', 'MuJoCo simulation');
        return;
      }
      simulateGraspInMujoco(grasp);
    },
    onToggleRecord: () => {
      if (recordTimer !== null) stopRecording();
      else startRecording();
    }
  });
  graspList.setGrasps(savedGrasps);

  // Helper to load gripper model
  const loadGripperModel = async () => {
    if (graspController) {
      return;
    }
    if (!viewer || !layout) return;

    try {
      layout.showLoading(true, `Loading ${GRIPPERS[activeGripperId].label} gripper assets...`);
      const gripperModel = await GripperLoader.loadGripper(GRIPPERS[activeGripperId]);

      graspController = new GraspController(
        gripperModel,
        viewer.getCamera(),
        viewer.getDomElement(),
        viewer.getScene(),
        viewer.getOrbitControls(),
        partController || undefined
      );

      // Connect CAD root if model is already loaded
      if (viewer.getModelRoot()) {
        graspController.setCadPartsRoot(viewer.getModelRoot());
        partController?.setCadPartsRoot(viewer.getModelRoot());
      }

      graspController.setOnRequestRender(() => {
        viewer?.requestRender();
      });

      graspController.setOnDraggingChanged((isDragging) => {
        viewer?.getSelectionManager().setIsDragging(isDragging);
      });

      (window as any).__graspController = graspController;
      (window as any).__viewer = viewer;
      (window as any).__partController = partController;

      // Hook state updates into UI
      graspController.setOnStateChange((state: GripperState) => {
        gripperPanel?.setGripperState(state);
        layout?.setTestGraspActive(state.isTesting);
        layout?.setTransformMode(graspController?.getTransformMode() || 'translate');
      });

      layout.setGripperBadge(true);
      layout.showLoading(false);

      if (lastClickedPoint) {
        gripperPanel.setCanSnap(true);
        layout.setCanSnap(true);
      }
    } catch (err: any) {
      layout.showLoading(false);
      layout.showError(
        err?.message || `Failed to load ${GRIPPERS[activeGripperId].label} gripper. Verify ${GRIPPERS[activeGripperId].baseUrl} assets.`
      );
    }
  };

  // 9. Connect Viewer events to UI
  viewer.setOnModelLoaded((metadata, parts) => {
    layout?.setLoadedFileName(metadata.fileName);
    partsList?.setParts(parts);
    propertiesPanel?.setModelMetadata(metadata);
    propertiesPanel?.setSelectionState({
      selectedPartId: null,
      selectedPartIndex: null,
      selectedPartName: null,
      clickedPoint: null
    });

    const modelRoot = viewer?.getModelRoot() ?? null;
    if (partController) {
      partController.setCadPartsRoot(modelRoot);
    }
    if (graspController) {
      graspController.setCadPartsRoot(modelRoot);
    }
  });

  viewer.setOnSelectionChanged((state) => {
    partsList?.setSelectedPart(state.selectedPartIndex);
    propertiesPanel?.setSelectionState(state);

    if (state.clickedPoint) {
      lastClickedPoint = state.clickedPoint;
      layout?.setCanSnap(true);
      gripperPanel?.setCanSnap(true);
    } else {
      lastClickedPoint = null;
      layout?.setCanSnap(false);
      gripperPanel?.setCanSnap(false);
    }
  });

  viewer.setOnError((errorMsg) => {
    layout?.showError(errorMsg);
  });

  viewer.setOnLoading((isLoading, message) => {
    layout?.showLoading(isLoading, message);
  });

  // 10. Auto-load initial CAD model and Gripper
  try {
    const urlParams = new URLSearchParams(window.location.search);
    const initialUrl =
      urlParams.get('step') ||
      urlParams.get('file') ||
      urlParams.get('cad') ||
      '/skf_cam_roller_305703_c-2z_0.stp';

    const initialName = initialUrl.split('/').pop()?.split('?')[0] || 'model.step';

    layout.showLoading(true, `Initializing OpenCascade WASM & loading ${initialName}...`);
    const resp = await fetch(initialUrl);
    if (resp.ok) {
      const buffer = await resp.arrayBuffer();
      await loadCADBuffer(buffer, initialName);
    } else {
      const fallbackUrl = '/samples/as1_pe_203.stp';
      const fallbackResp = await fetch(fallbackUrl);
      if (fallbackResp.ok) {
        const buffer = await fallbackResp.arrayBuffer();
        await loadCADBuffer(buffer, 'as1_pe_203.stp');
      } else {
        layout.showLoading(false);
      }
    }

    // Auto-load gripper so it is immediately ready
    await loadGripperModel();
  } catch {
    layout.showLoading(false);
  }
});
