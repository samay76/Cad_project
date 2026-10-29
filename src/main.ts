import { AppLayout } from './ui/Layout';
import { Viewer } from './viewer/Viewer';
import { PartsListView } from './ui/PartsList';
import { PropertiesPanel } from './ui/PropertiesPanel';
import './styles.css';

document.addEventListener('DOMContentLoaded', async () => {
  const appElement = document.getElementById('app');
  if (!appElement) throw new Error('App root element #app not found.');

  // Initialize master layout
  const layout = new AppLayout(appElement, {
    onFileSelected: async (file: File) => {
      try {
        layout.showLoading(true, `Reading ${file.name}...`);
        const buffer = await file.arrayBuffer();
        layout.setLoadedFileName(file.name);
        await viewer.loadFile(buffer, file.name);
      } catch (err: any) {
        layout.showLoading(false);
        layout.showError(err?.message || `Failed to read file ${file.name}`);
      }
    },
    onSampleSelected: async (sampleName: string, sampleUrl: string) => {
      try {
        layout.showLoading(true, `Loading sample ${sampleName}...`);
        const response = await fetch(sampleUrl);
        if (!response.ok) {
          throw new Error(`Failed to fetch ${sampleName}: HTTP ${response.status}`);
        }
        const buffer = await response.arrayBuffer();
        layout.setLoadedFileName(sampleName);
        await viewer.loadFile(buffer, sampleName);
      } catch (err: any) {
        layout.showLoading(false);
        layout.showError(err?.message || `Failed to load sample ${sampleName}`);
      }
    }
  });

  // Initialize 3D Viewer in viewport container
  const viewer = new Viewer(layout.viewportContainer);

  // Initialize Properties Panel in right sidebar
  const propertiesPanel = new PropertiesPanel(layout.rightSidebarContainer);

  // Initialize Parts List in left sidebar
  const partsList = new PartsListView(layout.leftSidebarContainer, {
    onResetView: () => {
      viewer.resetView();
    },
    onFitToModel: () => {
      viewer.fitToModel();
    },
    onToggleWireframe: () => {
      return viewer.toggleWireframe();
    },
    onViewPreset: (preset) => {
      viewer.setCameraView(preset);
    },
    onPartSelect: (index) => {
      viewer.selectPartByIndex(index);
    }
  });

  // Connect Viewer events to UI
  viewer.setOnModelLoaded((metadata, parts) => {
    layout.setLoadedFileName(metadata.fileName);
    partsList.setParts(parts);
    propertiesPanel.setModelMetadata(metadata);
    propertiesPanel.setSelectionState({
      selectedPartId: null,
      selectedPartIndex: null,
      selectedPartName: null,
      clickedPoint: null
    });
  });

  viewer.setOnSelectionChanged((state) => {
    partsList.setSelectedPart(state.selectedPartIndex);
    propertiesPanel.setSelectionState(state);
  });

  viewer.setOnError((errorMsg) => {
    layout.showError(errorMsg);
  });

  viewer.setOnLoading((isLoading, message) => {
    layout.showLoading(isLoading, message);
  });

  // Auto-load default STEP assembly sample on startup
  try {
    layout.showLoading(true, 'Initializing OpenCascade WASM & loading default CAD model...');
    const defaultSampleUrl = '/samples/as1_pe_203.stp';
    const resp = await fetch(defaultSampleUrl);
    if (resp.ok) {
      const buffer = await resp.arrayBuffer();
      layout.setLoadedFileName('as1_pe_203.stp');
      await viewer.loadFile(buffer, 'as1_pe_203.stp');
    } else {
      layout.showLoading(false);
    }
  } catch {
    layout.showLoading(false);
  }
});
