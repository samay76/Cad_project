import { AppLayout } from './ui/Layout';
import { Viewer } from './viewer/Viewer';
import { PartsListView } from './ui/PartsList';
import { PropertiesPanel } from './ui/PropertiesPanel';
import { GripperPanel } from './ui/GripperPanel';
import { PartPanel } from './ui/PartPanel';
import { AutoPickPanel } from './ui/AutoPickPanel';
import { LogPanel } from './ui/LogPanel';
import { ActivityCard } from './ui/ActivityCard';
import { FixtureLibrary } from './ui/FixtureLibrary';
import { library } from './library/api';
import { friendly, friendlyEvent, INITIAL_PILLS, nextPills, type Pills } from './ui/friendly';
import { GripperLoader } from './gripper/GripperLoader';
import { GraspController } from './gripper/GraspController';
import { PartController } from './part/PartController';
import { AutoPickController } from './autopick/AutoPickController';
import { FixtureView } from './autopick/FixtureView';
import type { SavedGrasp, GripperState } from './types';
import {
  GRIPPERS,
  GRIPPER_ID,
  ROBOT_ID,
  STATION_NUMBER,
  applyGripperSpec
} from './config';
import './styles.css';
import './operator.css';

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
  let graspController: GraspController | null = null;
  let partController: PartController | null = null;
  let autoPick: AutoPickController | null = null;
  let fixtureView: FixtureView | null = null;
  let logPanel: LogPanel | null = null;
  let activity: ActivityCard | null = null;
  let fixtures: FixtureLibrary | null = null;

  // State
  let currentModelName = 'model.step';
  let currentModelBuffer: ArrayBuffer | null = null;
  const activeGripperId = GRIPPER_ID;
  applyGripperSpec(GRIPPERS[activeGripperId]);
  const activeRobotId = ROBOT_ID;

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
    // The part made for the pick fixture is shown sitting on it (as in the simulator), to teach on it there.
    fixtureView ??= new FixtureView(viewer);
    fixtureView
      .update(currentModelBuffer)
      .then((shown) => shown && logPanel?.add(`Pick fixture: ${fileName} is shown on the pick fixture, as it sits in the simulator.`, 'info'))
      .catch((err) => console.warn('Fixture view:', err));

    const modelRoot = viewer.getModelRoot();
    if (partController) {
      partController.setCadPartsRoot(modelRoot);
    }
    if (graspController) {
      graspController.setCadPartsRoot(modelRoot);
    }
    autoPick?.showCell(); // the part is loaded into the workspace: table, EC66 at home, room
    activity?.add({ text: `${fileName} loaded`, kind: 'ok' });
    autoPick?.refresh();
  };

  // Helper to restore grasp (places gripper at T_world_part * T_part_grasp)
  const restoreGrasp = async (grasp: SavedGrasp) => {
    if (!graspController) {
      layout?.showError('Please load the gripper first.');
      return;
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

  // 1. Initialize master layout
  layout = new AppLayout(appElement, {
    onFileSelected: async (file: File) => {
      if (!layout) return;
      // A file opened directly (File > Open Part, dropped on the view) is not a library part
      if (fixtures) fixtures.current = null;
      layout.setSelection(null, null);
      fixtureView?.setSource(null); // the config's pick fixture
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
    onTransformModeToggle: (mode) => {
      graspController?.setTransformMode(mode);
    },
    onTargetToggle: (target) => {
      graspController?.setActiveEditTarget(target);
      layout?.setActiveTarget(target);
      partPanel?.setActiveTarget(target);
    },
    // Stop robot: always sent, whatever the screen shows
    onStop: () => {
      if (autoPick) autoPick.stop();
      else fetch('/api/mujoco/stop', { method: 'POST' }).catch(() => undefined);
    },
    onResetView: () => viewer?.resetView(),
    onTopView: () => viewer?.setCameraView('top')
  });

  layout.setActiveGripper(activeGripperId);
  layout.setStation(STATION_NUMBER);

  // 2. Initialize 3D Viewer into viewport container
  viewer = new Viewer(layout.viewportContainer);

  // 3. Initialize Part Controller
  partController = new PartController();

  // 4. Initialize CAD Properties Panel in right sidebar
  propertiesPanel = new PropertiesPanel(layout.propertiesPanelContainer);

  // 5. Initialize Parts List in left sidebar
  partsList = new PartsListView(layout.partsListContainer, {
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
      partController?.bakeCurrentOrientationAsOrigin(() => autoPick?.clearPoints());
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
    onStepTranslate: (axis, delta) => {
      graspController?.stepTranslate(axis, delta);
      autoPick?.gripperMoved();
    },
    onStepRotate: (axis, delta) => {
      graspController?.stepRotate(axis, delta);
      autoPick?.gripperMoved();
    }
  });

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
    } catch (err: any) {
      layout.showLoading(false);
      layout.showError(
        err?.message || `Failed to load ${GRIPPERS[activeGripperId].label} gripper. Verify ${GRIPPERS[activeGripperId].baseUrl} assets.`
      );
    }
  };

  // Automatic pick & place: two marked points -> grasp -> planned motion -> preview -> execute
  logPanel = new LogPanel(layout.logContainer); // Settings > Engineer log: every raw message
  activity = new ActivityCard(layout.activityContainer); // "What happened": plain language
  let pills: Pills = INITIAL_PILLS;
  const setPills = (p: Pills) => {
    pills = p;
    layout?.setPills(p);
  };
  setPills(pills);
  // Messages that would be pop-ups: raw text to the engineer log, errors also to "What happened"
  layout.logSink = (text, kind) => {
    logPanel?.add(text, kind);
    if (kind === 'error') activity?.add(friendly(text.replace(/^[^:]{1,40}:\s*/, ''), 'error'));
  };
  // The app server answers? (the robot pill turns red with the fix when it does not)
  const checkServer = async () => {
    let up = false;
    try {
      up = (await fetch('/api/health', { cache: 'no-store' })).ok;
    } catch {
      up = false;
    }
    setPills(nextPills(pills, 'server', up ? 'ok' : 'error', '', false));
  };
  checkServer();
  window.setInterval(checkServer, 5000);

  // Fixture library (Load part step): fixtures of STEP parts, kept by the cell server
  const showSelection = () => {
    const names = fixtures?.currentNames();
    layout?.setSelection(names?.fixture ?? null, names?.part ?? null);
  };
  let restoring = false; // grip points being restored from the library: not saved back
  fixtures = new FixtureLibrary({
    onUsePart: async (fx, part) => {
      if (fixtures) fixtures.current = null; // nothing is saved to the old part while the new one loads
      fixtureView ??= new FixtureView(viewer!);
      fixtureView.setSource(fx.id); // the part is shown in this fixture's own CAD
      layout?.showLoading(true, `Opening ${part.name}…`);
      try {
        const buffer = await library.partFile(fx.id, part.id);
        await loadCADBuffer(buffer, part.fileName);
      } finally {
        layout?.showLoading(false);
      }
      if (part.grip?.points.length) {
        restoring = true;
        autoPick?.restorePoints(part.grip.mode, part.grip.points);
        restoring = false;
      }
      activity?.add({ text: `${part.name} from fixture ${fx.name} is ready${part.grip ? ', with its saved grip points' : ''}`, kind: 'ok' });
    },
    onNotice: (text, kind) => {
      activity?.add({ text, kind });
      logPanel?.add(`Fixture library: ${text}`, kind === 'error' ? 'error' : kind === 'ok' ? 'ok' : 'info');
    },
    onSelectionChanged: showSelection
  });

  const panel = new AutoPickPanel(
    layout.autoPickContainer,
    { stepper: layout.stepperContainer, chip: layout.viewChipContainer, settings: layout.settingsRobotContainer },
    {
      pickFile: () => layout?.pickFile(),
      handleFile: (file) => layout?.handleFile(file),
      renderFixtures: (el) => fixtures?.render(el)
    }
  );
  autoPick = new AutoPickController(panel, {
    viewer,
    getGraspController: () => graspController,
    getPart: () => ({ name: currentModelName, buffer: currentModelBuffer }),
    getGripperId: () => activeGripperId,
    getRobotId: () => activeRobotId,
    restoreGrasp,
    showError: (message, title) => logPanel?.add(`${title}: ${message}`, 'error'), // already in "What happened"
    log: (text, kind) => logPanel?.add(text, kind),
    onEvent: (job, phase, raw, realGripper) => {
      setPills(nextPills(pills, job, phase, raw, realGripper));
      if (phase !== 'start') activity?.add(friendlyEvent(job, phase, raw));
    },
    // Taught grip points are saved with the library part in use
    getFixtureId: () => fixtures?.currentPart()?.fixture.id ?? null,
    onPointsChanged: (mode, points) => {
      const cur = fixtures?.currentPart();
      if (!cur || restoring) return;
      const grip = points.length ? { mode, points } : null;
      fixtures!.setGrip(cur.part.id, grip);
      library
        .saveGrip(cur.fixture.id, cur.part.id, grip)
        .catch((e) => logPanel?.add(`Fixture library: grip points not saved: ${e?.message || e}`, 'error'));
    }
  });

  (window as any).__autoPick = autoPick; // for the browser test

  // 9. Connect Viewer events to UI
  viewer.setOnModelLoaded((metadata, parts) => {
    // The new model starts with nothing selected; a point on the previous part means nothing here.
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

    if (state.clickedPoint) autoPick?.markPoint(state.clickedPoint);
  });

  viewer.setOnError((errorMsg) => {
    layout?.showError(errorMsg);
  });

  viewer.setOnLoading((isLoading, message) => {
    layout?.showLoading(isLoading, message);
  });

  // Load the gripper on startup, but wait for the user to choose a CAD file.
  loadGripperModel()
    .then(() => graspController?.setVisible(false)) // you teach with the arm's own gripper (in the workspace)
    .catch(() => {
      // loadGripperModel reports failures through the UI.
    });
  autoPick.showCell(); // open straight into the workspace: table, EC66 at home, room
});
