/** Fixture library on the cell server (server/library.py): fixtures, their STEP parts, taught grip points. */
import { arrayBufferToBase64, type MarkedPoint } from '../autopick/api';

export interface SavedGrip {
  mode: 'grip' | 'touch';
  points: MarkedPoint[];
  savedAt?: string;
}

export interface LibraryPart {
  id: string;
  name: string;
  fileName: string;
  size: number;
  uploadedAt: string;
  grip: SavedGrip | null;
}

export interface FixtureFile {
  fileName: string;
  size: number;
  uploadedAt: string;
}

export interface Fixture {
  id: string;
  name: string;
  file: FixtureFile | null; // the fixture's own CAD (the nest); parts are seated on it
  parts: LibraryPart[];
  createdAt: string;
}

const call = async <T>(method: string, url: string, body?: unknown): Promise<T> => {
  const resp = await fetch(url, {
    method,
    headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body)
  });
  const res = await resp.json().catch(() => ({}));
  if (!resp.ok || res.ok === false) throw new Error(res.message || `The library could not be reached (HTTP ${resp.status}).`);
  return res as T;
};

const fx = (id: string) => `/api/library/fixtures/${encodeURIComponent(id)}`;
const part = (fid: string, pid: string) => `${fx(fid)}/parts/${encodeURIComponent(pid)}`;

export const STEP_FILE = /\.(step|stp)$/i;

export const library = {
  list: async () => (await call<{ fixtures: Fixture[] }>('GET', '/api/library')).fixtures,
  addFixture: async (name: string, file: File) => {
    if (!STEP_FILE.test(file.name)) throw new Error('Only STEP files can be added.');
    const fileBase64 = arrayBufferToBase64(await file.arrayBuffer());
    return (await call<{ fixture: Fixture }>('POST', '/api/library/fixtures', { name, fileName: file.name, fileBase64 })).fixture;
  },
  setFixtureFile: async (id: string, file: File) => {
    if (!STEP_FILE.test(file.name)) throw new Error('Only STEP files can be added.');
    const fileBase64 = arrayBufferToBase64(await file.arrayBuffer());
    return (await call<{ fixture: Fixture }>('PUT', `${fx(id)}/file`, { fileName: file.name, fileBase64 })).fixture;
  },
  renameFixture: async (id: string, name: string) => (await call<{ fixture: Fixture }>('PATCH', fx(id), { name })).fixture,
  deleteFixture: (id: string) => call('DELETE', fx(id)),
  addPart: async (fid: string, file: File) => {
    if (!STEP_FILE.test(file.name)) throw new Error('Only STEP files can be added.');
    const fileBase64 = arrayBufferToBase64(await file.arrayBuffer());
    return (await call<{ part: LibraryPart }>('POST', `${fx(fid)}/parts`, { fileName: file.name, fileBase64 })).part;
  },
  renamePart: async (fid: string, pid: string, name: string) =>
    (await call<{ part: LibraryPart }>('PATCH', part(fid, pid), { name })).part,
  saveGrip: (fid: string, pid: string, grip: SavedGrip | null) =>
    call('PATCH', part(fid, pid), grip ? { grip } : { clearGrip: true }),
  deletePart: (fid: string, pid: string) => call('DELETE', part(fid, pid)),
  partFile: async (fid: string, pid: string) => {
    const resp = await fetch(`${part(fid, pid)}/file`, { cache: 'no-store' });
    if (!resp.ok) throw new Error('The part file could not be read.');
    return resp.arrayBuffer();
  }
};
