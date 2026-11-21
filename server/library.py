"""
library.py - The fixture library: named fixtures, each a group of STEP parts, kept on this PC.

  <data_dir>/library/library.json     fixtures and their parts (written atomically)
  <data_dir>/library/parts/<id>.step  the uploaded part files
  <data_dir>/library/fixtures/<id>.step  each fixture's own CAD (the nest the parts sit in)

  GET    /api/library                                      { fixtures: [...] }
  POST   /api/library/fixtures                             { name, fileName, fileBase64 }   with the fixture's STEP
  PATCH  /api/library/fixtures/<fid>                       { name }
  PUT    /api/library/fixtures/<fid>/file                  { fileName, fileBase64 }   add / replace the fixture's STEP
  GET    /api/library/fixtures/<fid>/file                  the fixture's STEP
  DELETE /api/library/fixtures/<fid>                       also deletes its part files
  POST   /api/library/fixtures/<fid>/parts                 { fileName, fileBase64, name? }   STEP / STP only
  PATCH  /api/library/fixtures/<fid>/parts/<pid>           { name?, grip? }   grip: taught points, or null
  DELETE /api/library/fixtures/<fid>/parts/<pid>
  GET    /api/library/fixtures/<fid>/parts/<pid>/file      the STEP file

Fixture names are unique (case-insensitive); part names are unique within their fixture.
"""
import base64
import binascii
import json
import os
import threading
import time
import uuid
from typing import Any, Dict, List, Literal, Optional

from fastapi import HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

STEP_FILE = ('.step', '.stp')
ID_LEN = 12


class NameBody(BaseModel):
    name: str = Field(min_length=1, max_length=80)


class FixtureBody(BaseModel):
    name: str = Field(min_length=1, max_length=80)
    fileName: Optional[str] = Field(None, max_length=200)
    fileBase64: Optional[str] = None


class FileBody(BaseModel):
    fileName: str = Field(min_length=1, max_length=200)
    fileBase64: str = Field(min_length=1)


class PartBody(BaseModel):
    fileName: str = Field(min_length=1, max_length=200)
    fileBase64: str = Field(min_length=1)
    name: Optional[str] = Field(None, max_length=80)


class Vec3(BaseModel):
    x: float
    y: float
    z: float


class GripPoint(BaseModel):
    point: Vec3
    normal: Vec3


class Grip(BaseModel):
    mode: Literal['grip', 'touch']
    points: List[GripPoint] = Field(max_length=2)


class PartPatch(BaseModel):
    name: Optional[str] = Field(None, min_length=1, max_length=80)
    grip: Optional[Grip] = None
    clearGrip: bool = False


def _now():
    return time.strftime('%Y-%m-%dT%H:%M:%S%z')


def _new_id():
    return uuid.uuid4().hex[:ID_LEN]


def _clean(name):
    name = ' '.join((name or '').split())
    if not name:
        raise HTTPException(400, 'A name is required.')
    return name


class Library:
    def __init__(self, data_dir):
        self.root = os.path.join(data_dir, 'library')
        self.parts_dir = os.path.join(self.root, 'parts')
        self.fixtures_dir = os.path.join(self.root, 'fixtures')
        self.file = os.path.join(self.root, 'library.json')
        self.lock = threading.Lock()

    # ---------------- storage ----------------
    def _load(self) -> Dict[str, Any]:
        if not os.path.exists(self.file):
            return {'fixtures': []}
        with open(self.file) as f:
            return json.load(f)

    def _save(self, doc):
        os.makedirs(self.root, exist_ok=True)
        tmp = self.file + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(doc, f, indent=1)
        os.replace(tmp, self.file)

    @staticmethod
    def _fixture(doc, fid):
        for fx in doc['fixtures']:
            if fx['id'] == fid:
                return fx
        raise HTTPException(404, 'This fixture no longer exists.')

    @staticmethod
    def _part(fx, pid):
        for p in fx['parts']:
            if p['id'] == pid:
                return p
        raise HTTPException(404, 'This part no longer exists.')

    @staticmethod
    def _unique_fixture(doc, name, skip=None):
        if any(fx['name'].lower() == name.lower() and fx['id'] != skip for fx in doc['fixtures']):
            raise HTTPException(409, f'A fixture named "{name}" already exists.')

    @staticmethod
    def _unique_part(fx, name, skip=None):
        if any(p['name'].lower() == name.lower() and p['id'] != skip for p in fx['parts']):
            raise HTTPException(409, f'This fixture already has a part named "{name}".')

    def _path(self, pid):
        return os.path.join(self.parts_dir, f'{pid}.step')

    def _fixture_file(self, fid):
        return os.path.join(self.fixtures_dir, f'{fid}.step')

    @staticmethod
    def _step(file_name, data_b64):
        """(clean file name, bytes) of an uploaded STEP file."""
        file_name = os.path.basename((file_name or '').strip())
        if not file_name.lower().endswith(STEP_FILE):
            raise HTTPException(400, 'Only STEP files can be added.')
        try:
            data = base64.b64decode(data_b64 or '', validate=True)
        except (binascii.Error, ValueError):
            raise HTTPException(400, 'The file could not be read.') from None
        if not data:
            raise HTTPException(400, 'The file is empty.')
        return file_name, data

    @staticmethod
    def _write(path, data):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'wb') as f:
            f.write(data)
        os.replace(tmp, path)

    def fixture_path(self, fid):
        """The fixture's own STEP file, or None (unknown fixture or no file yet)."""
        if not fid or not all(c in '0123456789abcdef' for c in fid):
            return None
        path = self._fixture_file(fid)
        with self.lock:
            known = any(fx['id'] == fid and fx.get('file') for fx in self._load()['fixtures'])
        return path if known and os.path.exists(path) else None

    def fixture_file_name(self, fid):
        with self.lock:
            for fx in self._load()['fixtures']:
                if fx['id'] == fid and fx.get('file'):
                    return fx['file']['fileName']
        return 'fixture.step'

    # ---------------- actions ----------------
    def list(self):
        with self.lock:
            return self._load()

    def add_fixture(self, name, file_name=None, data_b64=None):
        name = _clean(name)
        upload = self._step(file_name, data_b64) if (file_name or data_b64) else None
        with self.lock:
            doc = self._load()
            self._unique_fixture(doc, name)
            fx = {'id': _new_id(), 'name': name, 'file': None, 'parts': [], 'createdAt': _now()}
            if upload:
                self._write(self._fixture_file(fx['id']), upload[1])
                fx['file'] = {'fileName': upload[0], 'size': len(upload[1]), 'uploadedAt': _now()}
            doc['fixtures'].append(fx)
            self._save(doc)
            return fx

    def set_fixture_file(self, fid, file_name, data_b64):
        file_name, data = self._step(file_name, data_b64)
        with self.lock:
            doc = self._load()
            fx = self._fixture(doc, fid)
            self._write(self._fixture_file(fid), data)
            fx['file'] = {'fileName': file_name, 'size': len(data), 'uploadedAt': _now()}
            self._save(doc)
            return fx

    def fixture_file_response(self, fid):
        path = self.fixture_path(fid)
        if not path:
            raise HTTPException(404, 'This fixture has no CAD file yet.')
        return FileResponse(path, media_type='application/octet-stream', filename=self.fixture_file_name(fid),
                            headers={'Cache-Control': 'no-store'})

    def rename_fixture(self, fid, name):
        name = _clean(name)
        with self.lock:
            doc = self._load()
            fx = self._fixture(doc, fid)
            self._unique_fixture(doc, name, skip=fid)
            fx['name'] = name
            self._save(doc)
            return fx

    def delete_fixture(self, fid):
        with self.lock:
            doc = self._load()
            fx = self._fixture(doc, fid)
            doc['fixtures'].remove(fx)
            self._save(doc)
            try:
                os.remove(self._fixture_file(fid))
            except FileNotFoundError:
                pass
            for p in fx['parts']:
                try:
                    os.remove(self._path(p['id']))
                except FileNotFoundError:
                    pass

    def add_part(self, fid, file_name, data_b64, name=None):
        file_name, data = self._step(file_name, data_b64)
        name = _clean(name or os.path.splitext(file_name)[0])
        with self.lock:
            doc = self._load()
            fx = self._fixture(doc, fid)
            self._unique_part(fx, name)
            part = {'id': _new_id(), 'name': name, 'fileName': file_name, 'size': len(data),
                    'uploadedAt': _now(), 'grip': None}
            self._write(self._path(part['id']), data)
            fx['parts'].append(part)
            self._save(doc)
            return part

    def update_part(self, fid, pid, patch: PartPatch):
        with self.lock:
            doc = self._load()
            fx = self._fixture(doc, fid)
            part = self._part(fx, pid)
            if patch.name is not None:
                name = _clean(patch.name)
                self._unique_part(fx, name, skip=pid)
                part['name'] = name
            if patch.grip is not None:
                part['grip'] = {**patch.grip.model_dump(), 'savedAt': _now()}
            elif patch.clearGrip:
                part['grip'] = None
            self._save(doc)
            return part

    def delete_part(self, fid, pid):
        with self.lock:
            doc = self._load()
            fx = self._fixture(doc, fid)
            part = self._part(fx, pid)
            fx['parts'].remove(part)
            self._save(doc)
            try:
                os.remove(self._path(pid))
            except FileNotFoundError:
                pass

    def part_file(self, fid, pid):
        with self.lock:
            doc = self._load()
            part = self._part(self._fixture(doc, fid), pid)
        path = self._path(pid)
        if not os.path.exists(path):
            raise HTTPException(404, 'The part file is missing.')
        return FileResponse(path, media_type='application/octet-stream', filename=part['fileName'],
                            headers={'Cache-Control': 'no-store'})


def register(app, data_dir, body_of):
    """Adds the /api/library routes (before the server's catch-all /api route)."""
    lib = Library(data_dir)
    app.state.library = lib

    @app.get('/api/library')
    def library():
        return {'ok': True, **lib.list()}

    @app.post('/api/library/fixtures')
    async def add_fixture(request: Request):
        b = await body_of(request, FixtureBody)
        return {'ok': True, 'fixture': lib.add_fixture(b.name, b.fileName, b.fileBase64)}

    @app.put('/api/library/fixtures/{fid}/file')
    async def set_fixture_file(fid: str, request: Request):
        b = await body_of(request, FileBody)
        return {'ok': True, 'fixture': lib.set_fixture_file(fid, b.fileName, b.fileBase64)}

    @app.get('/api/library/fixtures/{fid}/file')
    def fixture_file(fid: str):
        return lib.fixture_file_response(fid)

    @app.patch('/api/library/fixtures/{fid}')
    async def rename_fixture(fid: str, request: Request):
        b = await body_of(request, NameBody)
        return {'ok': True, 'fixture': lib.rename_fixture(fid, b.name)}

    @app.delete('/api/library/fixtures/{fid}')
    def delete_fixture(fid: str):
        lib.delete_fixture(fid)
        return {'ok': True}

    @app.post('/api/library/fixtures/{fid}/parts')
    async def add_part(fid: str, request: Request):
        b = await body_of(request, PartBody)
        return {'ok': True, 'part': lib.add_part(fid, b.fileName, b.fileBase64, b.name)}

    @app.patch('/api/library/fixtures/{fid}/parts/{pid}')
    async def update_part(fid: str, pid: str, request: Request):
        b = await body_of(request, PartPatch)
        return {'ok': True, 'part': lib.update_part(fid, pid, b)}

    @app.delete('/api/library/fixtures/{fid}/parts/{pid}')
    def delete_part(fid: str, pid: str):
        lib.delete_part(fid, pid)
        return {'ok': True}

    @app.get('/api/library/fixtures/{fid}/parts/{pid}/file')
    def part_file(fid: str, pid: str):
        return lib.part_file(fid, pid)
