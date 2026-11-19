/**
 * Shows the pick fixture (config/cell.json pick_station) in the 3D view under the part made for it
 * (pick_station.seated_part), so the grasp points are taught with the part as it sits in the cell.
 *
 * Both STEPs are drawn in the same CAD frame (the simulator seats the part with that same fit), so the fixture
 * goes in at the identity: with the part at its starting pose, the part rests on it exactly as in MuJoCo.
 * The fixture is only scenery: it cannot be clicked (points land on the part behind it) and it is not the part.
 */
import * as THREE from 'three';
import type { Viewer } from '../viewer/Viewer';
import { loadStepFile } from '../viewer/loaders';

interface FixtureInfo {
  name: string;
  base64: string;
  seatedSha256: string;
  seatAll?: boolean; // pick_station.seat_all_parts: every part goes on the fixture (same mounting)
}

const FIXTURE_COLOR = 0x8c9fbf; // the blue-grey of the station in MuJoCo

const sha256 = async (buf: ArrayBuffer) =>
  Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', buf)))
    .map((b) => b.toString(16).padStart(2, '0'))
    .join('');

const base64ToBuffer = (b64: string) => {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out.buffer;
};

export class FixtureView {
  private group: THREE.Group | null = null;
  private loadedFor: string | null = null; // fixture name + sha it was built from

  constructor(private viewer: Viewer) {}

  /** True while the fixture is shown (the loaded part is the one made for it). */
  public get shown() {
    return this.group !== null;
  }

  /** Call after a part was loaded: shows the fixture under it if it is the fixture's part, else removes it. */
  public async update(partBuffer: ArrayBuffer | null): Promise<boolean> {
    let info: FixtureInfo | null = null;
    try {
      const resp = await fetch('/api/mujoco/fixture');
      const res = await resp.json();
      info = res?.ok ? res.fixture : null;
    } catch {
      info = null; // no bridge (static build): nothing to show
    }
    const partSha = partBuffer ? await sha256(partBuffer) : null;
    const match = !!info && !!partBuffer && (!!info.seatAll || partSha === info.seatedSha256);
    // The fixture loaded as the part itself (e.g. the scale): the part already is the fixture, draw it once.
    const isFixture = !!info && partSha === (await sha256(base64ToBuffer(info.base64)));
    if (!match || !info || isFixture) {
      this.remove();
      return false;
    }
    const key = `${info.name}:${info.base64.length}`; // the fixture's file, whichever part sits on it
    if (this.group && this.loadedFor === key) {
      this.viewer.getScene().add(this.group); // the part was reloaded: keep the same fixture
      this.place();
      return true;
    }
    this.remove();
    const result = await loadStepFile(base64ToBuffer(info.base64), info.name);
    const group = result.modelRoot;
    group.name = 'pick_fixture';
    group.userData = { isFixture: true };
    group.traverse((obj) => {
      obj.raycast = () => {}; // not clickable: points are marked on the part only
      if ((obj as THREE.Mesh).isMesh) {
        const mesh = obj as THREE.Mesh;
        mesh.userData = { isFixture: true };
        mesh.material = new THREE.MeshStandardMaterial({
          color: FIXTURE_COLOR,
          roughness: 0.5,
          metalness: 0.2,
          side: THREE.DoubleSide
        });
      }
    });
    this.group = group;
    this.loadedFor = key;
    this.viewer.getScene().add(group);
    this.place();
    return true;
  }

  public remove() {
    if (this.group) this.viewer.getScene().remove(this.group);
    this.group = null;
    this.viewer.setGroundZ(null);
    this.viewer.requestRender();
  }

  /** Ground grid under the fixture's base, and the view framing part + fixture. */
  private place() {
    if (!this.group) return;
    const box = new THREE.Box3().setFromObject(this.group);
    const root = this.viewer.getModelRoot();
    if (root) box.union(new THREE.Box3().setFromObject(root));
    this.viewer.setGroundZ(box.min.z);
    this.viewer.fitToBox(box, false);
    this.viewer.requestRender();
  }
}
