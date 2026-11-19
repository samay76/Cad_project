/**
 * The live view's look: real-world materials for the cell's parts and an industrial room around the table
 * (concrete floor, yellow/black safety marking, wall panels, room light with shadows). Used by LiveView.
 * Everything here is in the sim world frame, metres (LiveView scales it into the 3D view).
 */
import * as THREE from 'three';

export interface GeomInfo {
  type: string;
  bodyName?: string;
  geomName?: string;
  material?: string;
  rgba: number[];
}

const mat = (color: number | string, roughness: number, metalness: number, extra: Partial<THREE.MeshStandardMaterialParameters> = {}) =>
  new THREE.MeshStandardMaterial({ color, roughness, metalness, side: THREE.DoubleSide, ...extra });

/** What a geom really looks like: Elite EC66 white, Robotiq 2F-85 black / dark red / aluminium, black table. */
export function materialFor(g: GeomInfo, look: Record<string, string> = {}): THREE.MeshStandardMaterial {
  const body = g.bodyName ?? '';
  const name = g.geomName ?? '';
  const [r, gr, b, a] = g.rgba;
  if (body === 'cad_part') return mat(look.part_color ?? '#1f6fe5', 0.55, 0.05);
  if (name.startsWith('pickstation')) return mat(look.fixture_color ?? '#1f6fe5', 0.55, 0.05);
  if (name.startsWith('table_base')) return mat(0x2b2e33, 0.5, 0.45);           // black anodised aluminium
  if (name === 'boundary') return mat(0xf2c200, 0.6, 0.0);                       // yellow tape
  if (name.startsWith('box_')) return mat(0x2c3238, 0.7, 0.05);                  // tray
  if (name === 'coupling') return mat(0x3a3d42, 0.4, 0.6);
  if (/^(base_link|link\d)$/.test(body)) {
    return body === 'link6' ? mat(0xb7bbc1, 0.35, 0.6) : mat(0xeff1f3, 0.32, 0.05);   // white arm, aluminium flange
  }
  if (body.startsWith('g_')) {
    if (g.material === 'g_metal') return mat(0xc3c7cc, 0.3, 0.75);              // aluminium linkages
    if (body === 'g_base') return mat(0x3b1b22, 0.45, 0.2);                      // Robotiq body, dark red
    return mat(0x141416, 0.55, 0.1);                                             // black finger covers
  }
  return mat(new THREE.Color(r, gr, b).getHex(), 0.55, 0.15, { transparent: a < 1, opacity: a });
}

const canvasTexture = (size: number, draw: (c: CanvasRenderingContext2D, s: number) => void, repeat: [number, number]) => {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  draw(canvas.getContext('2d')!, size);
  const t = new THREE.CanvasTexture(canvas);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(...repeat);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
};

/** Polished concrete, 2 m slabs with expansion joints. */
const concrete = (repeat: number) =>
  canvasTexture(512, (c, s) => {
    c.fillStyle = '#8c9094';
    c.fillRect(0, 0, s, s);
    for (let i = 0; i < 9000; i++) {
      const v = 120 + Math.floor(Math.random() * 40);
      c.fillStyle = `rgba(${v},${v + 2},${v + 5},${0.08 + Math.random() * 0.12})`;
      c.fillRect(Math.random() * s, Math.random() * s, 1 + Math.random() * 3, 1 + Math.random() * 3);
    }
    c.strokeStyle = 'rgba(40,42,46,0.55)';
    c.lineWidth = 3;
    c.strokeRect(1.5, 1.5, s - 3, s - 3);
  }, [repeat, repeat]);

/** Yellow / black hazard stripes. */
const hazard = (length: number) =>
  canvasTexture(256, (c, s) => {
    c.fillStyle = '#f2c200';
    c.fillRect(0, 0, s, s);
    c.fillStyle = '#16171a';
    for (let x = -s; x < 2 * s; x += s / 2) {
      c.beginPath();
      c.moveTo(x, 0);
      c.lineTo(x + s / 4, 0);
      c.lineTo(x + s / 4 + s, s);
      c.lineTo(x + s, s);
      c.fill();
    }
  }, [length / 0.25, 1]);

/** Factory wall: light grey panels with seams, a darker lower band and a yellow guard line. */
const wallTexture = (width: number) =>
  canvasTexture(512, (c, s) => {
    c.fillStyle = '#cfd3d8';
    c.fillRect(0, 0, s, s);
    c.fillStyle = '#5f656c';
    c.fillRect(0, s * 0.66, s, s * 0.34);
    c.fillStyle = '#f2c200';
    c.fillRect(0, s * 0.64, s, s * 0.025);
    c.strokeStyle = 'rgba(70,75,82,0.45)';
    c.lineWidth = 3;
    c.strokeRect(1.5, 1.5, s - 3, s * 0.64);
  }, [width / 1.2, 1]);

/** Closed roller door: horizontal grey slats. */
const rollerDoor = () =>
  canvasTexture(256, (c, s) => {
    c.fillStyle = '#9aa1a8';
    c.fillRect(0, 0, s, s);
    for (let y = 0; y < s; y += 16) {
      c.fillStyle = '#7f868d';
      c.fillRect(0, y, s, 3);
      c.fillStyle = '#b4bac0';
      c.fillRect(0, y + 3, s, 2);
    }
  }, [1, 3]);

/**
 * The room around the cell. `table` = the table's box (sim world, m), `floorZ` = floor height (m).
 * Returns the room (add it to the cell group) and the lights (add them to the 3D view's scene, in its frame).
 */
export function buildRoom(table: THREE.Box3, floorZ: number) {
  const room = new THREE.Group();
  room.name = 'cell_room';
  const c = table.getCenter(new THREE.Vector3());
  const size = table.getSize(new THREE.Vector3());

  const floorSize = 6.0;     // the room's floor (walls 3 m from the table centre)
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(floorSize, floorSize),
    new THREE.MeshStandardMaterial({ map: concrete(floorSize / 2), roughness: 0.85, metalness: 0.0 })
  );
  floor.position.set(c.x, c.y, floorZ);
  floor.receiveShadow = true;
  room.add(floor);

  // safety marking around the table, 0.45 m off its edges
  const m = 0.45;
  const w = 0.08;
  const sx = size.x + 2 * m;
  const sy = size.y + 2 * m;
  const strip = (len: number, x: number, y: number, rot: number) => {
    const s = new THREE.Mesh(new THREE.PlaneGeometry(len, w), new THREE.MeshStandardMaterial({ map: hazard(len), roughness: 0.6 }));
    s.position.set(x, y, floorZ + 0.002);
    s.rotation.z = rot;
    s.receiveShadow = true;
    room.add(s);
  };
  strip(sx + w, c.x, c.y - sy / 2, 0);
  strip(sx + w, c.x, c.y + sy / 2, 0);
  strip(sy, c.x - sx / 2, c.y, Math.PI / 2);
  strip(sy, c.x + sx / 2, c.y, Math.PI / 2);

  // four walls around the cell, one-sided (they face inwards: seen from outside they do not block the view), and a
  // gate with a closed roller door in the wall on the -y side
  const half = 3.0;          // m from the table centre to each wall
  const height = 3.4;
  const up = new THREE.Vector3(0, 0, 1);
  const panel = (len: number, h: number, mid: THREE.Vector3, inward: THREE.Vector3, material: THREE.Material) => {
    const g = new THREE.Mesh(new THREE.PlaneGeometry(len, h), material);
    const along = new THREE.Vector3().crossVectors(up, inward);
    g.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(along, up, inward));
    g.position.copy(mid);
    g.receiveShadow = true;
    room.add(g);
    return g;
  };
  const wallMat = (len: number) => new THREE.MeshStandardMaterial({ map: wallTexture(len), roughness: 0.9, side: THREE.FrontSide });
  const z0 = floorZ;
  const sides: [THREE.Vector3, THREE.Vector3][] = [
    [new THREE.Vector3(c.x - half, c.y, 0), new THREE.Vector3(1, 0, 0)],   // behind the robot
    [new THREE.Vector3(c.x + half, c.y, 0), new THREE.Vector3(-1, 0, 0)],
    [new THREE.Vector3(c.x, c.y + half, 0), new THREE.Vector3(0, -1, 0)],
  ];
  for (const [p, n] of sides) panel(2 * half, height, p.clone().setZ(z0 + height / 2), n, wallMat(2 * half));
  // the gate wall (-y): wall | gate | wall, a lintel above the gate, a roller door and a yellow/black frame
  const gateW = 1.8;
  const gateH = 2.5;
  const gx = c.x + 0.6;                       // gate centre along the wall
  const wy = c.y - half;
  const inward = new THREE.Vector3(0, 1, 0);
  const leftLen = gx - gateW / 2 - (c.x - half);
  const rightLen = c.x + half - (gx + gateW / 2);
  panel(leftLen, height, new THREE.Vector3(c.x - half + leftLen / 2, wy, z0 + height / 2), inward, wallMat(leftLen));
  panel(rightLen, height, new THREE.Vector3(c.x + half - rightLen / 2, wy, z0 + height / 2), inward, wallMat(rightLen));
  panel(gateW, height - gateH, new THREE.Vector3(gx, wy, z0 + gateH + (height - gateH) / 2), inward,
        new THREE.MeshStandardMaterial({ color: 0xcfd3d8, roughness: 0.9, side: THREE.FrontSide }));
  panel(gateW, gateH, new THREE.Vector3(gx, wy - 0.06, z0 + gateH / 2), inward,
        new THREE.MeshStandardMaterial({ map: rollerDoor(), roughness: 0.55, metalness: 0.5, side: THREE.FrontSide }));
  const post = (x: number) => {
    const b = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, gateH + 0.12), new THREE.MeshStandardMaterial({ map: hazard(0.5), roughness: 0.6 }));
    b.position.set(x, wy + 0.04, z0 + (gateH + 0.12) / 2);
    room.add(b);
  };
  post(gx - gateW / 2 - 0.06);
  post(gx + gateW / 2 + 0.06);
  const lintel = new THREE.Mesh(new THREE.BoxGeometry(gateW + 0.24, 0.12, 0.12), new THREE.MeshStandardMaterial({ map: hazard(gateW), roughness: 0.6 }));
  lintel.position.set(gx, wy + 0.04, z0 + gateH + 0.06);
  room.add(lintel);
  // a hatched floor area in front of the gate (keep clear)
  const keepClear = new THREE.Mesh(new THREE.PlaneGeometry(gateW, 1.0), new THREE.MeshStandardMaterial({ map: hazard(gateW), roughness: 0.7, transparent: true, opacity: 0.35 }));
  keepClear.position.set(gx, wy + 0.5, z0 + 0.002);
  room.add(keepClear);

  // light: a soft sky / ground fill and a ceiling light that casts shadows onto the table and the floor
  const hemi = new THREE.HemisphereLight(0xf5f7fb, 0x55595e, 1.1);
  hemi.up.set(0, 0, 1);
  hemi.position.set(c.x, c.y, floorZ + 4);
  const sun = new THREE.DirectionalLight(0xfff6ea, 2.2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.6;
  const target = new THREE.Object3D();
  return { room, hemi, sun, target, center: c };
}
