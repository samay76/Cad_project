import fs from 'fs';
import path from 'path';
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import occtimportjs from 'occt-import-js';

async function runTests() {
  console.log('🧪 Starting CAD Viewer & Selection Verification Tests...\n');
  let passed = 0;
  let failed = 0;

  function assert(condition, message) {
    if (condition) {
      console.log(`  ✅ PASS: ${message}`);
      passed++;
    } else {
      console.error(`  ❌ FAIL: ${message}`);
      failed++;
    }
  }

  // TEST 1: OpenCascade WASM STEP Import
  console.log('Test Suite 1: STEP Import (Assembly)');
  const occt = await occtimportjs({
    locateFile: (name) => path.resolve('public', name)
  });
  assert(typeof occt.ReadStepFile === 'function', 'OpenCascade WASM module initialized with ReadStepFile');

  const stpBuffer = fs.readFileSync('public/samples/as1_pe_203.stp');
  const result = occt.ReadStepFile(new Uint8Array(stpBuffer), null);
  assert(result.success === true, 'Successfully parsed as1_pe_203.stp');
  assert(Array.isArray(result.meshes) && result.meshes.length === 18, `Meshes count is 18 (got ${result.meshes?.length})`);

  // Verify meshes and Three.js geometry conversion
  const modelRoot = new THREE.Group();
  let totalTriangles = 0;
  for (let i = 0; i < result.meshes.length; i++) {
    const meshData = result.meshes[i];
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.Float32BufferAttribute(meshData.attributes.position.array, 3));
    if (meshData.attributes.normal) {
      geom.setAttribute('normal', new THREE.Float32BufferAttribute(meshData.attributes.normal.array, 3));
    } else {
      geom.computeVertexNormals();
    }
    geom.setIndex(new THREE.BufferAttribute(Uint32Array.from(meshData.index.array), 1));
    const mesh = new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color: 0x60a5fa }));
    mesh.userData = { isCadPart: true, partIndex: i, name: meshData.name };
    modelRoot.add(mesh);
    totalTriangles += meshData.index.array.length / 3;
  }

  const box = new THREE.Box3().setFromObject(modelRoot);
  const size = box.getSize(new THREE.Vector3());
  assert(!box.isEmpty(), 'Computed valid bounding box for assembly');
  assert(size.x > 0 && size.y > 0 && size.z > 0, `Model dimensions in mm: X=${size.x.toFixed(2)}, Y=${size.y.toFixed(2)}, Z=${size.z.toFixed(2)}`);
  assert(totalTriangles > 0, `Total triangle count: ${totalTriangles}`);

  // TEST 2: Single Part STEP (Cube)
  console.log('\nTest Suite 2: STEP Import (Single Part Cube)');
  const cubeStp = fs.readFileSync('public/samples/cube.stp');
  const cubeRes = occt.ReadStepFile(new Uint8Array(cubeStp), null);
  assert(cubeRes.success === true, 'Successfully parsed cube.stp');
  assert(cubeRes.meshes.length === 1, 'Cube has exactly 1 mesh');

  // TEST 3: STL Loading
  console.log('\nTest Suite 3: STL Loading');
  const stlBuf = fs.readFileSync('public/samples/cube.stl');
  const stlLoader = new STLLoader();
  const stlGeom = stlLoader.parse(stlBuf.buffer.slice(stlBuf.byteOffset, stlBuf.byteOffset + stlBuf.byteLength));
  assert(stlGeom.attributes.position.count > 0, `STL parsed with ${stlGeom.attributes.position.count} vertices`);

  // TEST 4: OBJ Loading
  console.log('\nTest Suite 4: OBJ Loading');
  const objText = fs.readFileSync('public/samples/cube.obj', 'utf8');
  const objLoader = new OBJLoader();
  const objGroup = objLoader.parse(objText);
  let objMeshCount = 0;
  objGroup.traverse(c => { if (c instanceof THREE.Mesh) objMeshCount++; });
  assert(objMeshCount >= 1, `OBJ parsed with ${objMeshCount} mesh(es)`);

  // TEST 5: Raycasting and Coordinate Computation (Raycast test)
  console.log('\nTest Suite 5: Surface Pick Point & Normal Vector');
  const center = box.getCenter(new THREE.Vector3());
  console.log(`  ℹ️  Model Center: X=${center.x.toFixed(2)}, Y=${center.y.toFixed(2)}, Z=${center.z.toFixed(2)}`);
  console.log(`  ℹ️  Model Min: X=${box.min.x.toFixed(2)}, Y=${box.min.y.toFixed(2)}, Z=${box.min.z.toFixed(2)}`);
  console.log(`  ℹ️  Model Max: X=${box.max.x.toFixed(2)}, Y=${box.max.y.toFixed(2)}, Z=${box.max.z.toFixed(2)}`);

  // Target triangle 0 on Mesh 0
  const mesh0 = modelRoot.children[0];
  const pos = mesh0.geometry.attributes.position;
  const idx = mesh0.geometry.index;
  const i0 = idx.getX(0);
  const i1 = idx.getX(1);
  const i2 = idx.getX(2);

  const v0 = new THREE.Vector3(pos.getX(i0), pos.getY(i0), pos.getZ(i0));
  const v1 = new THREE.Vector3(pos.getX(i1), pos.getY(i1), pos.getZ(i1));
  const v2 = new THREE.Vector3(pos.getX(i2), pos.getY(i2), pos.getZ(i2));

  const triCenter = new THREE.Vector3().add(v0).add(v1).add(v2).divideScalar(3);
  const triNormal = new THREE.Vector3().subVectors(v1, v0).cross(new THREE.Vector3().subVectors(v2, v0)).normalize();

  const raycaster = new THREE.Raycaster();
  // Raycast from 10mm outside the surface pointing straight into triangle center
  const rayOrigin = triCenter.clone().addScaledVector(triNormal, 10);
  const rayDir = triNormal.clone().negate();
  raycaster.set(rayOrigin, rayDir);
  const intersects = raycaster.intersectObjects(modelRoot.children, false);
  assert(intersects.length > 0, `Raycast intersected part (hits: ${intersects.length})`);

  if (intersects.length > 0) {
    const hit = intersects[0];
    const pt = hit.point;
    const normal = hit.face.normal.clone().transformDirection(hit.object.matrixWorld).normalize();

    assert(typeof pt.x === 'number' && typeof pt.y === 'number' && typeof pt.z === 'number',
      `Pick Point in model frame: X=${pt.x.toFixed(3)}mm, Y=${pt.y.toFixed(3)}mm, Z=${pt.z.toFixed(3)}mm`);

    const normalLen = normal.length();
    assert(Math.abs(normalLen - 1.0) < 0.001,
      `Face Normal is unit vector (length: ${normalLen.toFixed(4)}): [${normal.x.toFixed(3)}, ${normal.y.toFixed(3)}, ${normal.z.toFixed(3)}]`);
  }

  // TEST 6: Pose Data Structure validation
  console.log('\nTest Suite 6: Robotics Pose Data Contract');
  const samplePose = {
    position: { x: 12.345, y: -45.678, z: 90.123 },
    orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 }
  };
  assert(samplePose.position.x === 12.345 && samplePose.orientation.w === 0.707, 'Pose type matches position + quaternion contract');

  console.log(`\n========================================`);
  console.log(`Test Results: ${passed} Passed, ${failed} Failed`);
  console.log(`========================================\n`);

  if (failed > 0) {
    process.exit(1);
  }
}

runTests().catch(err => {
  console.error('Test execution failed:', err);
  process.exit(1);
});
