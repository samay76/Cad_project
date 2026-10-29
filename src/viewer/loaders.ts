import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import type { CADModelMetadata, PartInfo, BoundingBoxInfo } from '../types';

let occtInstance: any = null;

/**
 * Initializes and caches the OpenCascade WASM module
 */
export async function getOcct(): Promise<any> {
  if (!occtInstance) {
    let initFn = (window as any).occtimportjs;
    if (!initFn) {
      try {
        const mod = await import('occt-import-js');
        initFn = mod.default || mod;
      } catch {
        // Handled below
      }
    }

    if (typeof initFn !== 'function') {
      throw new Error('occtimportjs library not loaded. Ensure /occt-import-js.js is loaded in the browser.');
    }

    occtInstance = await initFn({
      locateFile: (name: string) => {
        if (name.endsWith('.wasm')) {
          return '/occt-import-js.wasm';
        }
        return name;
      }
    });
  }
  return occtInstance;
}

// Industrial CAD color palette for distinct parts
const CAD_PALETTE = [
  0x60a5fa, // Soft blue
  0x94a3b8, // Slate silver
  0x34d399, // Emerald
  0xfbbf24, // Amber
  0xa78bfa, // Violet
  0x38bdf8, // Sky blue
  0xf87171, // Coral red
  0x4ade80, // Light green
  0xf472b6, // Pink
  0x2dd4bf, // Teal
];

export interface LoadedCADResult {
  modelRoot: THREE.Group;
  parts: PartInfo[];
  metadata: CADModelMetadata;
  selectableMeshes: THREE.Mesh[];
}

/**
 * Traverses OpenCascade scene graph hierarchy to resolve meaningful part names
 */
function extractPartNamesFromRoot(root: any, meshCount: number): string[] {
  const names: string[] = new Array(meshCount).fill('');
  const counts: Record<string, number> = {};

  function traverse(node: any, path: string[]) {
    if (!node) return;
    const nodeName = typeof node.name === 'string' ? node.name.trim() : '';
    const currentPath = nodeName ? [...path, nodeName] : path;

    if (Array.isArray(node.meshes)) {
      for (const meshIdx of node.meshes) {
        if (meshIdx >= 0 && meshIdx < meshCount) {
          const baseName = currentPath.length > 0 ? currentPath[currentPath.length - 1] : `Part ${meshIdx + 1}`;
          names[meshIdx] = baseName;
        }
      }
    }

    if (Array.isArray(node.children)) {
      for (const child of node.children) {
        traverse(child, currentPath);
      }
    }
  }

  if (root) {
    traverse(root, []);
  }

  // Format duplicate names with index (e.g. Bracket #1, Bracket #2)
  return names.map((name, i) => {
    const raw = name || `Part ${i + 1}`;
    counts[raw] = (counts[raw] || 0) + 1;
    return counts[raw] > 1 ? `${raw} (${counts[raw]})` : raw;
  });
}

/**
 * Loads and triangulates a STEP / STP / IGES file via occt-import-js
 */
export async function loadStepFile(
  buffer: ArrayBuffer,
  fileName: string
): Promise<LoadedCADResult> {
  const occt = await getOcct();
  const fileBuffer = new Uint8Array(buffer);

  let result: any;
  const isIges = fileName.toLowerCase().endsWith('.iges') || fileName.toLowerCase().endsWith('.igs');
  if (isIges && typeof occt.ReadIgesFile === 'function') {
    result = occt.ReadIgesFile(fileBuffer, null);
  } else {
    result = occt.ReadStepFile(fileBuffer, null);
  }

  if (!result || !result.success || !Array.isArray(result.meshes) || result.meshes.length === 0) {
    throw new Error('Failed to parse STEP file: invalid format or no 3D geometry found.');
  }

  const modelRoot = new THREE.Group();
  modelRoot.name = fileName;
  const selectableMeshes: THREE.Mesh[] = [];
  const parts: PartInfo[] = [];

  const partNames = extractPartNamesFromRoot(result.root, result.meshes.length);
  let totalTriangles = 0;
  let totalVertices = 0;

  for (let i = 0; i < result.meshes.length; i++) {
    const meshData = result.meshes[i];
    if (!meshData.attributes?.position?.array || !meshData.index?.array) {
      continue;
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute(meshData.attributes.position.array, 3)
    );

    if (meshData.attributes.normal?.array) {
      geometry.setAttribute(
        'normal',
        new THREE.Float32BufferAttribute(meshData.attributes.normal.array, 3)
      );
    } else {
      geometry.computeVertexNormals();
    }

    const indices = Uint32Array.from(meshData.index.array);
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));

    // Determine part color
    let partColor: THREE.Color;
    if (meshData.color && Array.isArray(meshData.color) && meshData.color.length >= 3) {
      partColor = new THREE.Color(meshData.color[0], meshData.color[1], meshData.color[2]);
    } else {
      partColor = new THREE.Color(CAD_PALETTE[i % CAD_PALETTE.length]);
    }

    // Material setup
    const material = new THREE.MeshStandardMaterial({
      color: partColor,
      roughness: 0.35,
      metalness: 0.15,
      side: THREE.DoubleSide
    });

    const mesh = new THREE.Mesh(geometry, material);
    const partName = partNames[i] || meshData.name || `Part ${i + 1}`;
    mesh.name = partName;

    // Feature edge outlines for distinct CAD presentation
    let edgesGroup: THREE.LineSegments | null = null;
    try {
      const edgesGeom = new THREE.EdgesGeometry(geometry, 28);
      const edgeMaterial = new THREE.LineBasicMaterial({
        color: 0x334155,
        linewidth: 1,
        transparent: true,
        opacity: 0.6
      });
      edgesGroup = new THREE.LineSegments(edgesGeom, edgeMaterial);
      edgesGroup.renderOrder = 1;
      mesh.add(edgesGroup);
    } catch {
      // Ignore edge extraction error on non-manifold geometries
    }

    const triCount = meshData.index.array.length / 3;
    const vertCount = meshData.attributes.position.array.length / 3;
    totalTriangles += triCount;
    totalVertices += vertCount;

    const partId = `step_part_${i}`;
    mesh.userData = {
      isCadPart: true,
      partId,
      partIndex: i,
      partName,
      triangleCount: triCount,
      vertexCount: vertCount,
      baseColor: partColor.clone(),
      edges: edgesGroup
    };

    modelRoot.add(mesh);
    selectableMeshes.push(mesh);

    parts.push({
      id: partId,
      index: i,
      name: partName,
      triangleCount: triCount,
      vertexCount: vertCount,
      visible: true,
      color: `#${partColor.getHexString()}`,
      meshIndex: i
    });
  }

  const boundingBox = computeBoundingBox(modelRoot);

  const metadata: CADModelMetadata = {
    fileName,
    fileSize: buffer.byteLength,
    fileType: 'step',
    partsCount: parts.length,
    triangleCount: totalTriangles,
    vertexCount: totalVertices,
    boundingBox
  };

  return { modelRoot, parts, metadata, selectableMeshes };
}

/**
 * Loads an STL file (binary or ASCII)
 */
export async function loadStlFile(
  buffer: ArrayBuffer,
  fileName: string
): Promise<LoadedCADResult> {
  const loader = new STLLoader();
  const geometry = loader.parse(buffer);

  if (!geometry.attributes.position) {
    throw new Error('Failed to parse STL file: missing vertex positions.');
  }

  if (!geometry.attributes.normal) {
    geometry.computeVertexNormals();
  }

  const modelRoot = new THREE.Group();
  modelRoot.name = fileName;

  const partColor = new THREE.Color(0x94a3b8);
  const material = new THREE.MeshStandardMaterial({
    color: partColor,
    roughness: 0.35,
    metalness: 0.15,
    side: THREE.DoubleSide
  });

  const mesh = new THREE.Mesh(geometry, material);
  const cleanName = fileName.replace(/\.[^/.]+$/, '') || 'STL Model';
  mesh.name = cleanName;

  let edgesGroup: THREE.LineSegments | null = null;
  try {
    const edgesGeom = new THREE.EdgesGeometry(geometry, 28);
    const edgeMaterial = new THREE.LineBasicMaterial({
      color: 0x334155,
      linewidth: 1,
      transparent: true,
      opacity: 0.6
    });
    edgesGroup = new THREE.LineSegments(edgesGeom, edgeMaterial);
    edgesGroup.renderOrder = 1;
    mesh.add(edgesGroup);
  } catch {
    // Ignore edges geometry error
  }

  const triCount = geometry.index
    ? geometry.index.count / 3
    : geometry.attributes.position.count / 3;
  const vertCount = geometry.attributes.position.count;

  const partId = 'stl_part_0';
  mesh.userData = {
    isCadPart: true,
    partId,
    partIndex: 0,
    partName: cleanName,
    triangleCount: triCount,
    vertexCount: vertCount,
    baseColor: partColor.clone(),
    edges: edgesGroup
  };

  modelRoot.add(mesh);

  const parts: PartInfo[] = [
    {
      id: partId,
      index: 0,
      name: cleanName,
      triangleCount: triCount,
      vertexCount: vertCount,
      visible: true,
      color: `#${partColor.getHexString()}`,
      meshIndex: 0
    }
  ];

  const boundingBox = computeBoundingBox(modelRoot);

  const metadata: CADModelMetadata = {
    fileName,
    fileSize: buffer.byteLength,
    fileType: 'stl',
    partsCount: 1,
    triangleCount: triCount,
    vertexCount: vertCount,
    boundingBox
  };

  return {
    modelRoot,
    parts,
    metadata,
    selectableMeshes: [mesh]
  };
}

/**
 * Loads an OBJ file (Wavefront OBJ text format)
 */
export async function loadObjFile(
  buffer: ArrayBuffer,
  fileName: string
): Promise<LoadedCADResult> {
  const text = new TextDecoder('utf-8').decode(buffer);
  const loader = new OBJLoader();
  const objGroup = loader.parse(text);

  const modelRoot = new THREE.Group();
  modelRoot.name = fileName;
  const selectableMeshes: THREE.Mesh[] = [];
  const parts: PartInfo[] = [];

  let totalTriangles = 0;
  let totalVertices = 0;
  let partIndex = 0;

  objGroup.traverse((child) => {
    if (child instanceof THREE.Mesh) {
      const geometry = child.geometry;
      if (!geometry.attributes.normal) {
        geometry.computeVertexNormals();
      }

      const partColor = new THREE.Color(CAD_PALETTE[partIndex % CAD_PALETTE.length]);
      const material = new THREE.MeshStandardMaterial({
        color: partColor,
        roughness: 0.35,
        metalness: 0.15,
        side: THREE.DoubleSide
      });

      child.material = material;
      const partName = child.name || `Part ${partIndex + 1}`;
      child.name = partName;

      let edgesGroup: THREE.LineSegments | null = null;
      try {
        const edgesGeom = new THREE.EdgesGeometry(geometry, 28);
        const edgeMaterial = new THREE.LineBasicMaterial({
          color: 0x334155,
          linewidth: 1,
          transparent: true,
          opacity: 0.6
        });
        edgesGroup = new THREE.LineSegments(edgesGeom, edgeMaterial);
        edgesGroup.renderOrder = 1;
        child.add(edgesGroup);
      } catch {
        // Ignore edges error
      }

      const triCount = geometry.index
        ? geometry.index.count / 3
        : geometry.attributes.position.count / 3;
      const vertCount = geometry.attributes.position.count;
      totalTriangles += triCount;
      totalVertices += vertCount;

      const partId = `obj_part_${partIndex}`;
      child.userData = {
        isCadPart: true,
        partId,
        partIndex,
        partName,
        triangleCount: triCount,
        vertexCount: vertCount,
        baseColor: partColor.clone(),
        edges: edgesGroup
      };

      selectableMeshes.push(child);
      parts.push({
        id: partId,
        index: partIndex,
        name: partName,
        triangleCount: triCount,
        vertexCount: vertCount,
        visible: true,
        color: `#${partColor.getHexString()}`,
        meshIndex: partIndex
      });

      partIndex++;
    }
  });

  if (selectableMeshes.length === 0) {
    throw new Error('Failed to parse OBJ file: no meshes found in file.');
  }

  // Transfer meshes to modelRoot
  for (const mesh of selectableMeshes) {
    modelRoot.add(mesh);
  }

  const boundingBox = computeBoundingBox(modelRoot);

  const metadata: CADModelMetadata = {
    fileName,
    fileSize: buffer.byteLength,
    fileType: 'obj',
    partsCount: parts.length,
    triangleCount: totalTriangles,
    vertexCount: totalVertices,
    boundingBox
  };

  return { modelRoot, parts, metadata, selectableMeshes };
}

/**
 * Dispatches file loading based on filename extension
 */
export async function loadCADFile(
  buffer: ArrayBuffer,
  fileName: string
): Promise<LoadedCADResult> {
  const lowerName = fileName.toLowerCase();
  if (lowerName.endsWith('.step') || lowerName.endsWith('.stp') || lowerName.endsWith('.iges') || lowerName.endsWith('.igs')) {
    return loadStepFile(buffer, fileName);
  } else if (lowerName.endsWith('.stl')) {
    return loadStlFile(buffer, fileName);
  } else if (lowerName.endsWith('.obj')) {
    return loadObjFile(buffer, fileName);
  } else {
    throw new Error(`Unsupported file extension for "${fileName}". Please select a .step, .stp, .stl, or .obj file.`);
  }
}

/**
 * Computes bounding box dimensions and center in model coordinates (mm)
 */
export function computeBoundingBox(object: THREE.Object3D): BoundingBoxInfo {
  const box = new THREE.Box3().setFromObject(object);

  if (box.isEmpty()) {
    return {
      min: { x: 0, y: 0, z: 0 },
      max: { x: 0, y: 0, z: 0 },
      size: { x: 0, y: 0, z: 0 },
      center: { x: 0, y: 0, z: 0 }
    };
  }

  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());

  return {
    min: { x: box.min.x, y: box.min.y, z: box.min.z },
    max: { x: box.max.x, y: box.max.y, z: box.max.z },
    size: { x: size.x, y: size.y, z: size.z },
    center: { x: center.x, y: center.y, z: center.z }
  };
}
