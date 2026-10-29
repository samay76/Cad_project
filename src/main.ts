import { AppLayout } from './ui/Layout';
import { Viewer } from './viewer/Viewer';
import { PartsListView } from './ui/PartsList';
import { PropertiesPanel } from './ui/PropertiesPanel';
import { GripperPanel } from './ui/GripperPanel';
import { GraspListView } from './ui/GraspList';
import { GripperLoader } from './gripper/GripperLoader';
import { GraspController } from './gripper/GraspController';
import type { SavedGrasp, ClickedPointInfo, GripperState } from './types';
import './styles.css';

document.addEventListener('DOMContentLoaded', async () => {
  const appElement = document.getElementById('app');
  if (!appElement) throw new Error('App root element #app not found.');

  // Pre-declare component instances to avoid Temporal Dead Zone (TDZ)
  let viewer: Viewer | null = null;
  let layout: AppLayout | null = null;
  let partsList: PartsListView | null = null;
  let propertiesPanel: PropertiesPanel | null = null;
  let gripperPanel: GripperPanel | null = null;
  let graspList: GraspListView | null = null;
  let graspController: GraspController | null = null;

  // State
  let lastClickedPoint: ClickedPointInfo | null = null;
  let currentModelName = 'model.step';
  let savedGrasps: SavedGrasp[] = [];

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

    // Update CAD root in grasp controller for collision checking
    if (graspController) {
      graspController.setCadPartsRoot(viewer.getModelRoot());
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
      lastClickedPoint.point,
      lastClickedPoint.normal
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
      tcpPose: state.pose,
      rpy: state.rpy,
      openingWidth: state.openingWidth,
      preGraspDistance: state.preGraspDistance,
      tcpOffset: state.tcpOffset,
      createdAt: Date.now(),
      partName: currentModelName
    };

    savedGrasps.push(newGrasp);
    saveGraspsToStorage();
    graspList?.setGrasps(savedGrasps);
    graspList?.setSelectedGraspId(newGrasp.id);
  };

  // Helper to restore grasp
  const restoreGrasp = (grasp: SavedGrasp) => {
    if (!graspController) {
      layout?.showError('Please load the gripper first.');
      return;
    }
    graspController.setNumericPose(grasp.tcpPose.position, grasp.rpy);
    graspController.setOpeningWidth(grasp.openingWidth);
    graspController.setTcpOffset(grasp.tcpOffset);
    graspController.setPreGraspDistance(grasp.preGraspDistance);
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

  // Helper to import grasps from JSON
  const importGraspsJSON = (imported: SavedGrasp[]) => {
    if (!Array.isArray(imported) || imported.length === 0) {
      layout?.showError('Imported file does not contain valid grasp poses.');
      return;
    }
    // Validate grasps have tcpPose and rpy
    const valid = imported.filter(
      (g) => g && g.tcpPose && g.tcpPose.position && g.tcpPose.orientation
    );
    if (valid.length === 0) {
      layout?.showError('No valid grasp poses found in imported file.');
      return;
    }
    savedGrasps = [...savedGrasps, ...valid];
    saveGraspsToStorage();
    graspList?.setGrasps(savedGrasps);
    // Restore the first imported grasp
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
    onTransformModeToggle: (mode) => {
      graspController?.setTransformMode(mode);
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

  // 2. Initialize 3D Viewer into viewport container
  viewer = new Viewer(layout.viewportContainer);

  // 3. Initialize CAD Properties Panel in right sidebar
  propertiesPanel = new PropertiesPanel(layout.propertiesPanelContainer);

  // 4. Initialize Parts List in left sidebar
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

  // 5. Initialize Gripper Pose Panel in right sidebar
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
    }
  });

  // 6. Initialize Grasps List in left sidebar
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
    }
  });
  graspList.setGrasps(savedGrasps);

  // Helper to load gripper model
  const loadGripperModel = async () => {
    if (graspController) {
      // Gripper is already loaded
      return;
    }
    if (!viewer || !layout) return;

    try {
      layout.showLoading(true, 'Loading Robotiq 2F-85 Gripper assets...');
      const gripperModel = await GripperLoader.loadGripper();

      graspController = new GraspController(
        gripperModel,
        viewer.getCamera(),
        viewer.getDomElement(),
        viewer.getScene(),
        viewer.getOrbitControls()
      );

      // Connect CAD root if model is already loaded
      if (viewer.getModelRoot()) {
        graspController.setCadPartsRoot(viewer.getModelRoot());
      }

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
        err?.message || 'Failed to load Robotiq 2F-85 gripper. Verify /gripper/ assets.'
      );
    }
  };

  // 7. Connect Viewer events to UI
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

    if (graspController) {
      graspController.setCadPartsRoot(viewer?.getModelRoot() ?? null);
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

  // 8. Auto-load initial CAD model and Gripper
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

    // Auto-load gripper so it is immediately ready for positioning
    await loadGripperModel();
  } catch {
    layout.showLoading(false);
  }
});
