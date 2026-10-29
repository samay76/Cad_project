import { AppLayout } from './ui/Layout';
import { Viewer } from './viewer/Viewer';
import { PartsListView } from './ui/PartsList';
import { PropertiesPanel } from './ui/PropertiesPanel';
import './styles.css';

document.addEventListener('DOMContentLoaded', async () => {
  const appElement = document.getElementById('app');
  if (!appElement) throw new Error('App root element #app not found.');

  // Pre-declare references to avoid Temporal Dead Zone (TDZ) ReferenceError
  let viewer: Viewer | null = null;
  let layout: AppLayout | null = null;
  let partsList: PartsListView | null = null;
  let propertiesPanel: PropertiesPanel | null = null;

  // Safe CAD loader helper
  const loadCADBuffer = async (buffer: ArrayBuffer, fileName: string) => {
    if (!viewer || !layout) {
      throw new Error('3D Viewer is still initializing. Please try again in a moment.');
    }
    layout.setLoadedFileName(fileName);
    await viewer.loadFile(buffer, fileName);
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
    }
  });

  // 2. Initialize 3D Viewer immediately into the layout viewport container
  viewer = new Viewer(layout.viewportContainer);

  // 3. Initialize Properties Panel in right sidebar
  propertiesPanel = new PropertiesPanel(layout.rightSidebarContainer);

  // 4. Initialize Parts List in left sidebar
  partsList = new PartsListView(layout.leftSidebarContainer, {
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

  // 5. Connect Viewer events to UI
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
  });

  viewer.setOnSelectionChanged((state) => {
    partsList?.setSelectedPart(state.selectedPartIndex);
    propertiesPanel?.setSelectionState(state);
  });

  viewer.setOnError((errorMsg) => {
    layout?.showError(errorMsg);
  });

  viewer.setOnLoading((isLoading, message) => {
    layout?.showLoading(isLoading, message);
  });

  // 6. Auto-load initial model (check query param or load user file / default assembly)
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
      // Fallback to sample if initialUrl fails
      const fallbackUrl = '/samples/as1_pe_203.stp';
      const fallbackResp = await fetch(fallbackUrl);
      if (fallbackResp.ok) {
        const buffer = await fallbackResp.arrayBuffer();
        await loadCADBuffer(buffer, 'as1_pe_203.stp');
      } else {
        layout.showLoading(false);
      }
    }
  } catch {
    layout.showLoading(false);
  }
});
