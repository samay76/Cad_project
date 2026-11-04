import * as THREE from 'three';
import { AppLayout } from './ui/Layout';
import { Viewer } from './viewer/Viewer';
import { PartsListView } from './ui/PartsList';
import { PropertiesPanel } from './ui/PropertiesPanel';
import { GripperPanel } from './ui/GripperPanel';
import { PartPanel } from './ui/PartPanel';
import { GraspListView } from './ui/GraspList';
import { CameraFeedPanel } from './ui/CameraFeedPanel';
import { AutoPickPanel } from './ui/AutoPickPanel';
import { GripperLoader } from './gripper/GripperLoader';
import { GraspController } from './gripper/GraspController';
import { PartController } from './part/PartController';
import { AutoPickController } from './autopick/AutoPickController';
import { arrayBufferToBase64 } from './autopick/api';
import type { SavedGrasp, ClickedPointInfo, GripperState } from './types';
import {
  GRIPPERS,
  ROBOTS,
  applyGripperSpec,
  getStoredGripperId,
  getStoredRobotId,
  storeRobotId,
  type GripperId
} from './config';
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
  let autoPick: AutoPickController | null = null;

  // State
  let lastClickedPoint: ClickedPointInfo | null = null;
  let currentModelName = 'model.step';
  let currentModelBuffer: ArrayBuffer | null = null;
  let savedGrasps: SavedGrasp[] = [];
  let activeGripperId: GripperId = getStoredGripperId();
  applyGripperSpec(GRIPPERS[activeGripperId]);
  let activeRobotId = getStoredRobotId();

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
    autoPick?.clearPoints();

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
    const spec = GRIPPERS[id];
    if (!graspController) {
      activeGripperId = id;
      applyGripperSpec(spec);
      autoPick?.invalidate();
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
      autoPick?.invalidate();
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
            layout?.showInfo('MuJoCo simulation finished', '');
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

    layout?.showInfo(
      'MuJoCo simulation',
      `Starting simulator: ${ROBOTS[activeRobotId].label} + ${GRIPPERS[grasp.gripperId ?? '2f85'].label} (first run converts the STEP mesh, ~30 s)...`
    );
    try {
      const resp = await fetch('/api/mujoco/simulate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          grasp,
          robotId: activeRobotId,
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
    onLoadGripperClick: async () => {
      await loadGripperModel();
    },
    onGripperSelect: (id) => {
      selectGripper(id);
    },
    onRobotSelect: (id) => {
      activeRobotId = id;
      storeRobotId(id);
      autoPick?.invalidate();
      layout?.setActiveRobot(id);
      layout?.showInfo('Robot arm', `${ROBOTS[id].label} selected: the pick & place in MuJoCo uses it.`);
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
  });

  layout.setActiveGripper(activeGripperId);
  layout.setActiveRobot(activeRobotId);

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
    autoPick?.partMoved();
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
      simulateGraspInMujoco(grasp);
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

  // Automatic pick & place: two marked points -> grasp -> planned motion -> preview -> execute
  autoPick = new AutoPickController(new AutoPickPanel(layout.autoPickContainer), {
    viewer,
    getGraspController: () => graspController,
    getPart: () => ({ name: currentModelName, buffer: currentModelBuffer }),
    getGripperId: () => activeGripperId,
    getRobotId: () => activeRobotId,
    getCameraPanel: () => cameraPanel,
    restoreGrasp,
    showError: (message, title) => layout?.showError(message, title)
  });

  // 9. Connect Viewer events to UI
  viewer.setOnModelLoaded((metadata, parts) => {
    // The new model starts with nothing selected; a point on the previous part means nothing here.
    lastClickedPoint = null;
    layout?.setCanSnap(false);
    gripperPanel?.setCanSnap(false);
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
      autoPick?.markPoint(state.clickedPoint);
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

  // Load the gripper on startup, but wait for the user to choose a CAD file.
  loadGripperModel().catch(() => {
    // loadGripperModel reports failures through the UI.
  });
});
