# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""Minimal glTF 2.0 (.glb) writer with PBR materials and mesh helpers.

Conventions: metres, +Y up, the car faces -Z (engine convention).
"""
import io
import json
import struct

import numpy as np
from PIL import Image


# ---------------------------------------------------------------- geometry
class Mesh:
    """Indexed triangle mesh: positions (N,3), normals (N,3), uvs (N,2), tris (M,3)."""

    col = None

    def __init__(self, pos, tris, uv=None, nrm=None, col=None):
        self.col = None if col is None else np.asarray(col, np.float64).reshape(-1, 4)
        self.pos = np.asarray(pos, np.float64).reshape(-1, 3)
        self.tris = np.asarray(tris, np.int64).reshape(-1, 3)
        self.uv = np.zeros((len(self.pos), 2)) if uv is None else np.asarray(uv, np.float64).reshape(-1, 2)
        self.nrm = nrm if nrm is None else np.asarray(nrm, np.float64).reshape(-1, 3)

    def copy(self):
        return Mesh(self.pos.copy(), self.tris.copy(), self.uv.copy(), None if self.nrm is None else self.nrm.copy(),
                    None if self.col is None else self.col.copy())

    def transformed(self, m=None, t=(0, 0, 0), s=None):
        out = self.copy()
        p = out.pos
        if s is not None:
            p = p * np.asarray(s)
        if m is not None:
            p = p @ np.asarray(m).T
        out.pos = p + np.asarray(t)
        if out.nrm is not None:
            n = out.nrm
            if s is not None:
                n = n / np.asarray(s)
            if m is not None:
                n = n @ np.asarray(m).T
            out.nrm = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
            if s is not None and np.prod(np.sign(s)) < 0:
                out.tris = out.tris[:, ::-1]
        elif s is not None and np.prod(np.sign(s)) < 0:
            out.tris = out.tris[:, ::-1]
        return out

    def flipped(self):
        out = self.copy()
        out.tris = out.tris[:, ::-1]
        if out.nrm is not None:
            out.nrm = -out.nrm
        return out

    def compute_normals(self, flat=False, angle_deg=None):
        if flat:
            if self.col is not None:
                self.col = self.col[self.tris].reshape(-1, 4)
            p = self.pos[self.tris].reshape(-1, 3)
            uv = self.uv[self.tris].reshape(-1, 2)
            tris = np.arange(len(p)).reshape(-1, 3)
            fn = face_normals(self.pos, self.tris)
            self.pos, self.uv, self.tris = p, uv, tris
            self.nrm = np.repeat(fn, 3, axis=0)
            return self
        fn = np.cross(self.pos[self.tris[:, 1]] - self.pos[self.tris[:, 0]], self.pos[self.tris[:, 2]] - self.pos[self.tris[:, 0]])
        n = np.zeros_like(self.pos)
        for k in range(3):
            np.add.at(n, self.tris[:, k], fn)
        self.nrm = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        return self


def face_normals(pos, tris):
    fn = np.cross(pos[tris[:, 1]] - pos[tris[:, 0]], pos[tris[:, 2]] - pos[tris[:, 0]])
    return fn / np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-12)


def merge(meshes):
    meshes = [m for m in meshes if m is not None and len(m.tris)]
    pos, tris, uv, nrm, col = [], [], [], [], []
    off = 0
    any_col = any(m.col is not None for m in meshes)
    for m in meshes:
        if m.nrm is None:
            m = m.copy().compute_normals()
        pos.append(m.pos); uv.append(m.uv); nrm.append(m.nrm); tris.append(m.tris + off)
        if any_col:
            col.append(m.col if m.col is not None else np.ones((len(m.pos), 4)))
        off += len(m.pos)
    if not meshes:
        return None
    return Mesh(np.vstack(pos), np.vstack(tris), np.vstack(uv), np.vstack(nrm), np.vstack(col) if any_col else None)


def grid_tris(rows, cols, closed_cols=False, closed_rows=False):
    """Triangles for a rows x cols vertex grid (row-major)."""
    t = []
    cmax = cols if closed_cols else cols - 1
    rmax = rows if closed_rows else rows - 1
    for r in range(rmax):
        r2 = (r + 1) % rows
        for c in range(cmax):
            c2 = (c + 1) % cols
            a, b, cc, d = r * cols + c, r * cols + c2, r2 * cols + c2, r2 * cols + c
            t.append((a, b, cc)); t.append((a, cc, d))
    return np.array(t, np.int64)


def box(size, center=(0, 0, 0), uv_scale=1.0):
    sx, sy, sz = np.asarray(size) / 2
    meshes = []
    faces = [
        ((1, 0, 0), (0, 0, -1), (0, 1, 0)), ((-1, 0, 0), (0, 0, 1), (0, 1, 0)),
        ((0, 1, 0), (1, 0, 0), (0, 0, -1)), ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
        ((0, 0, 1), (1, 0, 0), (0, 1, 0)), ((0, 0, -1), (-1, 0, 0), (0, 1, 0)),
    ]
    half = np.array([sx, sy, sz])
    for n, u, v in faces:
        n, u, v = map(np.array, (n, u, v))
        c = n * half
        du = u * np.abs(u @ half); dv = v * np.abs(v @ half)
        p = np.array([c - du - dv, c + du - dv, c + du + dv, c - du + dv]) + center
        lu = 2 * np.abs(u @ half) * uv_scale; lv = 2 * np.abs(v @ half) * uv_scale
        uv = np.array([[0, lv], [lu, lv], [lu, 0], [0, 0]])
        meshes.append(Mesh(p, [(0, 1, 2), (0, 2, 3)], uv, np.tile(n, (4, 1))))
    return merge(meshes)


def revolve(profile, segments=48, axis="x", uv_v=None, cap=False):
    """Revolve a (r, a) profile around an axis. profile: list of (radius, axial)."""
    prof = np.asarray(profile, float)
    ang = np.linspace(0, 2 * np.pi, segments + 1)
    rows = len(prof)
    pos, uv = [], []
    L = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(prof, axis=0), axis=1))])
    for i, (r, a) in enumerate(prof):
        for j, t in enumerate(ang):
            c, s = np.cos(t), np.sin(t)
            if axis == "x":
                pos.append((a, r * c, r * s))
            elif axis == "y":
                pos.append((r * c, a, r * s))
            else:
                pos.append((r * c, r * s, a))
            uv.append((j / segments, L[i] if uv_v is None else uv_v[i]))
    tris = grid_tris(rows, segments + 1)
    m = Mesh(pos, tris, uv)
    # Orientation: the surface normal is the profile tangent (dr, da) rotated to (da, -dr).
    k = int(np.argmax(np.linalg.norm(np.diff(prof, axis=0), axis=1)))
    dr, da = prof[k + 1] - prof[k]
    tri = m.tris[2 * (k * (segments + 1))]
    fn = np.cross(m.pos[tri[1]] - m.pos[tri[0]], m.pos[tri[2]] - m.pos[tri[0]])
    mid = m.pos[tri].mean(axis=0)
    ax = {"x": 0, "y": 1, "z": 2}[axis]
    radial = mid.copy(); radial[ax] = 0
    radial /= max(np.linalg.norm(radial), 1e-12)
    axial = np.zeros(3); axial[ax] = 1
    expected = radial * da - axial * dr
    if fn @ expected < 0:
        m.tris = m.tris[:, ::-1]
    m.compute_normals()
    return m


def cylinder(r, h, segments=24, axis="y", center=(0, 0, 0), caps=True):
    prof = [(r, -h / 2), (r, h / 2)]
    side = revolve(prof, segments, axis)
    # hard edges: side normals radial already
    parts = [side]
    if caps:
        for sgn in (-1, 1):
            ang = np.linspace(0, 2 * np.pi, segments, endpoint=False)
            ring = []
            for t in ang:
                c, s = np.cos(t) * r, np.sin(t) * r
                if axis == "x":
                    ring.append((sgn * h / 2, c, s))
                elif axis == "y":
                    ring.append((c, sgn * h / 2, s))
                else:
                    ring.append((c, s, sgn * h / 2))
            ctr = {"x": (sgn * h / 2, 0, 0), "y": (0, sgn * h / 2, 0), "z": (0, 0, sgn * h / 2)}[axis]
            p = np.vstack([ctr, ring])
            t = [(0, 1 + i, 1 + (i + 1) % segments) for i in range(segments)]
            uvs = np.vstack([[0.5, 0.5], 0.5 + 0.5 * np.c_[np.cos(ang), np.sin(ang)]])
            cm = Mesh(p, t, uvs)
            n = np.zeros(3); n["xyz".index(axis)] = sgn
            cm.compute_normals()
            if cm.nrm[0] @ n < 0:
                cm = cm.flipped(); cm.compute_normals()
            parts.append(cm)
    m = merge(parts)
    m.pos += np.asarray(center)
    # make sure the side faces outward
    return m


def fix_outward(m, center):
    """Flip triangles whose normal points towards `center` (closed convex-ish parts)."""
    fn = face_normals(m.pos, m.tris)
    c = m.pos[m.tris].mean(axis=1)
    bad = np.einsum("ij,ij->i", fn, c - np.asarray(center)) < 0
    m.tris[bad] = m.tris[bad][:, ::-1]
    m.nrm = None
    return m


def sweep(profile2d, path, up=(0, 1, 0), closed_profile=True, closed_path=False, uv_u_scale=1.0):
    """Sweep a 2D profile (in the plane right/up) along a 3D polyline path."""
    path = np.asarray(path, float)
    prof = np.asarray(profile2d, float)
    n = len(path)
    tangents = np.gradient(path, axis=0) if not closed_path else (np.roll(path, -1, 0) - np.roll(path, 1, 0))
    tangents /= np.maximum(np.linalg.norm(tangents, axis=1, keepdims=True), 1e-12)
    up = np.asarray(up, float)
    pos, uv = [], []
    seglen = np.linalg.norm(np.diff(path, axis=0), axis=1)
    L = np.concatenate([[0], np.cumsum(seglen)])
    plen = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(prof, axis=0), axis=1))])
    for i in range(n):
        t = tangents[i]
        r = np.cross(t, up); r /= max(np.linalg.norm(r), 1e-12)
        u = np.cross(r, t)
        for j, (a, b) in enumerate(prof):
            pos.append(path[i] + r * a + u * b)
            uv.append((L[i] * uv_u_scale, plen[j]))
    tris = grid_tris(n, len(prof), closed_cols=closed_profile, closed_rows=closed_path)
    m = Mesh(pos, tris, uv)
    return m


# ---------------------------------------------------------------- glTF writer
class GLB:
    def __init__(self, generator="car-simulator asset builder"):
        self.j = {"asset": {"version": "2.0", "generator": generator}, "scenes": [{"nodes": []}], "scene": 0,
                  "nodes": [], "meshes": [], "materials": [], "accessors": [], "bufferViews": [],
                  "buffers": [], "images": [], "textures": [], "samplers": [{"magFilter": 9729, "minFilter": 9987, "wrapS": 10497, "wrapT": 10497}]}
        self.bin = bytearray()
        self.ext_used = set()
        self._img_cache = {}

    def _view(self, data, target=None):
        while len(self.bin) % 4:
            self.bin.append(0)
        off = len(self.bin)
        self.bin += data
        v = {"buffer": 0, "byteOffset": off, "byteLength": len(data)}
        if target:
            v["target"] = target
        self.j["bufferViews"].append(v)
        return len(self.j["bufferViews"]) - 1

    def _acc(self, arr, comp, typ, target, minmax=False):
        view = self._view(arr.tobytes(), target)
        a = {"bufferView": view, "componentType": comp, "count": int(arr.shape[0]), "type": typ}
        if minmax:
            a["min"] = arr.min(axis=0).tolist(); a["max"] = arr.max(axis=0).tolist()
        self.j["accessors"].append(a)
        return len(self.j["accessors"]) - 1

    def image(self, img, name, jpeg=False):
        if name in self._img_cache:
            return self._img_cache[name]
        bio = io.BytesIO()
        if isinstance(img, np.ndarray):
            img = Image.fromarray(img)
        if jpeg:
            img.convert("RGB").save(bio, "JPEG", quality=88, optimize=True)
        else:
            img.save(bio, "PNG", optimize=True)
        view = self._view(bio.getvalue())
        self.j["images"].append({"name": name, "mimeType": "image/jpeg" if jpeg else "image/png", "bufferView": view})
        self.j["textures"].append({"sampler": 0, "source": len(self.j["images"]) - 1, "name": name})
        idx = len(self.j["textures"]) - 1
        self._img_cache[name] = idx
        return idx

    def material(self, name, color=(1, 1, 1, 1), metallic=0.0, roughness=0.5, base_tex=None, mr_tex=None,
                 normal_tex=None, normal_scale=1.0, emissive=(0, 0, 0), emissive_strength=None, alpha=None,
                 double=False, clearcoat=None, clearcoat_rough=0.03, extras=None, emissive_tex=None):
        pbr = {"baseColorFactor": list(color), "metallicFactor": metallic, "roughnessFactor": roughness}
        if base_tex is not None:
            pbr["baseColorTexture"] = {"index": base_tex}
        if mr_tex is not None:
            pbr["metallicRoughnessTexture"] = {"index": mr_tex}
        m = {"name": name, "pbrMetallicRoughness": pbr}
        if normal_tex is not None:
            m["normalTexture"] = {"index": normal_tex, "scale": normal_scale}
        if any(emissive):
            m["emissiveFactor"] = list(emissive)
        if emissive_tex is not None:
            m["emissiveTexture"] = {"index": emissive_tex}
        ext = {}
        if emissive_strength:
            ext["KHR_materials_emissive_strength"] = {"emissiveStrength": emissive_strength}
        if clearcoat:
            ext["KHR_materials_clearcoat"] = {"clearcoatFactor": clearcoat, "clearcoatRoughnessFactor": clearcoat_rough}
        if ext:
            m["extensions"] = ext
            self.ext_used.update(ext.keys())
        if alpha == "BLEND":
            m["alphaMode"] = "BLEND"
        elif alpha == "MASK":
            m["alphaMode"] = "MASK"; m["alphaCutoff"] = 0.5
        if double:
            m["doubleSided"] = True
        if extras:
            m["extras"] = extras
        self.j["materials"].append(m)
        return len(self.j["materials"]) - 1

    def mesh(self, name, prims):
        """prims: list of (Mesh, material_index)."""
        out = []
        for m, mat in prims:
            if m is None or len(m.tris) == 0:
                continue
            if m.nrm is None:
                m = m.copy().compute_normals()
            pos = m.pos.astype(np.float32); nrm = m.nrm.astype(np.float32); uv = m.uv.astype(np.float32)
            idx = m.tris.astype(np.uint32).reshape(-1)
            attrs = {"POSITION": self._acc(pos, 5126, "VEC3", 34962, True),
                     "NORMAL": self._acc(nrm, 5126, "VEC3", 34962),
                     "TEXCOORD_0": self._acc(uv, 5126, "VEC2", 34962)}
            if m.col is not None:
                attrs["COLOR_0"] = self._acc(m.col.astype(np.float32), 5126, "VEC4", 34962)
            ind = self._acc(idx, 5125, "SCALAR", 34963)
            out.append({"attributes": attrs, "indices": ind, "material": mat})
        self.j["meshes"].append({"name": name, "primitives": out})
        return len(self.j["meshes"]) - 1

    def node(self, name, mesh=None, t=None, r=None, s=None, children=None, extras=None, root=True):
        n = {"name": name}
        if mesh is not None:
            n["mesh"] = mesh
        if t is not None:
            n["translation"] = [float(x) for x in t]
        if r is not None:
            n["rotation"] = [float(x) for x in r]
        if s is not None:
            n["scale"] = [float(x) for x in s]
        if children:
            n["children"] = children
        if extras:
            n["extras"] = extras
        self.j["nodes"].append(n)
        i = len(self.j["nodes"]) - 1
        if root:
            self.j["scenes"][0]["nodes"].append(i)
        return i

    def save(self, path):
        if self.ext_used:
            self.j["extensionsUsed"] = sorted(self.ext_used)
        for k in ("images", "textures"):
            if not self.j[k]:
                del self.j[k]
        while len(self.bin) % 4:
            self.bin.append(0)
        self.j["buffers"] = [{"byteLength": len(self.bin)}]
        js = json.dumps(self.j, separators=(",", ":")).encode()
        while len(js) % 4:
            js += b" "
        total = 12 + 8 + len(js) + 8 + len(self.bin)
        with open(path, "wb") as f:
            f.write(struct.pack("<III", 0x46546C67, 2, total))
            f.write(struct.pack("<II", len(js), 0x4E4F534A)); f.write(js)
            f.write(struct.pack("<II", len(self.bin), 0x004E4942)); f.write(self.bin)
        return total
