#!/usr/bin/env python3
"""Convert URDF visual meshes into MuJoCo-loadable OBJ assets.

    python3 tools/champ_mesh_assets.py <robot> [--package-dir dir] [--force]

MuJoCo reads STL/OBJ/MSH only, and it cannot colour parts of one mesh
differently. The Unitree URDFs use Blender-exported COLLADA (.dae) files with
several materials per part, so every visual mesh is split into one OBJ per
material under mujoco/assets/<robot>/ together with manifest.json that records
the material colours. tools/generate_mjcf.py reads that manifest to emit the
<asset>/<geom type="mesh"> entries, so switching robots is still URDF + yaml.

The converter is self-contained (numpy + ElementTree): it applies the COLLADA
<unit> scale and the visual-scene node transforms (Blender writes the Y-up ->
Z-up rotation there), triangulates <polylist> polygons (fan), keeps the
per-vertex normals and resolves material -> effect -> diffuse colour. Binary
and ASCII STL files are converted too (single material, no colour).

Conversion is skipped when manifest.json already lists the source file with
the same SHA-1, so repeated generate_mjcf.py runs are cheap.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from champ_robot_files import (  # noqa: E402
    PACKAGE_NAME, SOURCE_PACKAGE_DIR, Urdf, load_urdf, resolve_robot_files,
)

MANIFEST_NAME = 'manifest.json'
MANIFEST_VERSION = 1
DEFAULT_RGBA = (0.7, 0.7, 0.7, 1.0)


@dataclass
class SubMesh:
    """Triangles of one material: positions (N,3), optional normals (M,3), faces (F,3,2)."""
    material: str
    rgba: Tuple[float, float, float, float]
    positions: np.ndarray
    normals: Optional[np.ndarray]
    faces: np.ndarray  # int64 (F, 3, 2): [position index, normal index or -1]


@dataclass
class AssetEntry:
    name: str  # MuJoCo mesh name
    file: str  # relative to the assets directory
    material: str
    rgba: List[float]
    triangles: int


@dataclass
class MeshAssets:
    directory: Path
    entries: Dict[str, List[AssetEntry]] = field(default_factory=dict)  # source rel path -> parts


# ---------------------------------------------------------------------------
# URDF mesh filename resolution
# ---------------------------------------------------------------------------

def resolve_mesh_path(filename: str, package_dir: Path) -> Path:
    """package://pkg/rel, file:///abs, $(find pkg)/rel or a plain path -> local file."""
    text = filename.strip()
    if text.startswith('package://'):
        pkg, _, rel = text[len('package://'):].partition('/')
        if pkg == PACKAGE_NAME:
            return package_dir / rel
        from champ_robot_files import _package_share_dir

        return Path(_package_share_dir(pkg)) / rel
    if text.startswith('file://'):
        text = text[len('file://'):]
    match = re.match(r'\$\(find\s+([A-Za-z0-9_]+)\)/?(.*)', text)
    if match:
        pkg, rel = match.group(1), match.group(2)
        if pkg == PACKAGE_NAME:
            return package_dir / rel
        from champ_robot_files import _package_share_dir

        return Path(_package_share_dir(pkg)) / rel
    path = Path(text)
    return path if path.is_absolute() else package_dir / path


def mesh_asset_stem(source: Path, package_dir: Path) -> str:
    """Stable MuJoCo-safe name from the mesh path relative to the package."""
    try:
        rel = source.resolve().relative_to(package_dir.resolve())
    except ValueError:
        rel = Path(source.name)
    if rel.parts and rel.parts[0] == 'meshes':
        rel = Path(*rel.parts[1:])
    stem = '_'.join(rel.with_suffix('').parts)
    return re.sub(r'[^A-Za-z0-9_]', '_', stem)


# ---------------------------------------------------------------------------
# COLLADA
# ---------------------------------------------------------------------------

def _strip_ns(tag: str) -> str:
    return tag.rsplit('}', 1)[-1]


def _floats(text: Optional[str]) -> np.ndarray:
    return np.array((text or '').split(), dtype=np.float64)


def _ints(text: Optional[str]) -> np.ndarray:
    return np.array((text or '').split(), dtype=np.int64)


def _node_matrix(node, ns: str) -> np.ndarray:
    """Compose <matrix>/<translate>/<rotate>/<scale> children in document order."""
    total = np.eye(4)
    for child in node:
        tag = _strip_ns(child.tag)
        if tag == 'matrix':
            m = _floats(child.text).reshape(4, 4)
        elif tag == 'translate':
            m = np.eye(4)
            m[:3, 3] = _floats(child.text)
        elif tag == 'rotate':
            x, y, z, deg = _floats(child.text)
            axis = np.array([x, y, z])
            n = np.linalg.norm(axis)
            m = np.eye(4)
            if n > 0:
                axis = axis / n
                a = math.radians(deg)
                k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                m[:3, :3] = np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)
        elif tag == 'scale':
            m = np.diag([*_floats(child.text), 1.0])
        else:
            continue
        total = total @ m
    return total


class ColladaFile:
    def __init__(self, path: Path) -> None:
        import xml.etree.ElementTree as ET

        self.path = path
        self.root = ET.parse(path).getroot()
        self.ns = self.root.tag[1:].split('}')[0] if self.root.tag.startswith('{') else ''
        self.unit = 1.0
        asset = self._find(self.root, 'asset')
        if asset is not None:
            unit = self._find(asset, 'unit')
            if unit is not None and 'meter' in unit.attrib:
                self.unit = float(unit.attrib['meter'])
        self.up_axis = 'Z_UP'
        if asset is not None:
            up = self._find(asset, 'up_axis')
            if up is not None and up.text:
                self.up_axis = up.text.strip()
        self.by_id = {el.attrib['id']: el for el in self.root.iter() if 'id' in el.attrib}
        self.effect_rgba = self._effect_colours()
        self.material_effect = {
            m.attrib['id']: (self._find(m, 'instance_effect').attrib.get('url', '').lstrip('#')
                             if self._find(m, 'instance_effect') is not None else '')
            for m in self.root.iter(self._tag('material')) if 'id' in m.attrib
        }

    # -- xml helpers ------------------------------------------------------------
    def _tag(self, name: str) -> str:
        return f'{{{self.ns}}}{name}' if self.ns else name

    def _find(self, el, name: str):
        return el.find(self._tag(name))

    def _findall(self, el, name: str):
        return el.findall(self._tag(name))

    def _ref(self, url: str):
        return self.by_id.get(url.lstrip('#'))

    # -- materials --------------------------------------------------------------
    def _effect_colours(self) -> Dict[str, Tuple[float, float, float, float]]:
        out: Dict[str, Tuple[float, float, float, float]] = {}
        for effect in self.root.iter(self._tag('effect')):
            rgba = None
            for diffuse in effect.iter(self._tag('diffuse')):
                colour = self._find(diffuse, 'color')
                if colour is not None and colour.text:
                    values = _floats(colour.text)
                    if len(values) >= 3:
                        rgba = tuple(float(v) for v in (list(values[:4]) + [1.0])[:4])
                    break
            out[effect.attrib.get('id', '')] = rgba or DEFAULT_RGBA
        return out

    def material_rgba(self, material_id: str) -> Tuple[float, float, float, float]:
        effect = self.material_effect.get(material_id, '')
        return self.effect_rgba.get(effect, DEFAULT_RGBA)

    def material_key(self, material_id: str) -> str:
        """Effect id when known (the Unitree URDFs name their <material> after it)."""
        return self.material_effect.get(material_id) or material_id

    # -- geometry ---------------------------------------------------------------
    def _source_array(self, source_id: str) -> np.ndarray:
        source = self._ref(source_id)
        if source is None:
            raise ValueError(f'{self.path}: missing source {source_id}')
        array = self._find(source, 'float_array')
        values = _floats(array.text)
        technique = self._find(source, 'technique_common')
        stride = 3
        if technique is not None:
            accessor = self._find(technique, 'accessor')
            if accessor is not None:
                stride = int(accessor.attrib.get('stride', '3'))
        return values.reshape(-1, stride)[:, :3]

    def _vertex_positions(self, vertices_id: str) -> str:
        vertices = self._ref(vertices_id)
        for inp in self._findall(vertices, 'input'):
            if inp.attrib.get('semantic') == 'POSITION':
                return inp.attrib['source']
        raise ValueError(f'{self.path}: <vertices> without POSITION')

    def _primitives(self, mesh) -> List[Tuple[str, Dict[str, str], np.ndarray, np.ndarray, Dict[str, int], int]]:
        """(material symbol, {semantic: source}, vcount, indices, {semantic: offset}, stride)."""
        out = []
        for prim in mesh:
            kind = _strip_ns(prim.tag)
            if kind not in ('triangles', 'polylist', 'polygons'):
                continue
            inputs = self._findall(prim, 'input')
            sources: Dict[str, str] = {}
            offsets: Dict[str, int] = {}
            stride = 0
            for inp in inputs:
                semantic = inp.attrib['semantic']
                offset = int(inp.attrib.get('offset', '0'))
                stride = max(stride, offset + 1)
                if semantic == 'VERTEX':
                    sources['POSITION'] = self._vertex_positions(inp.attrib['source'])
                    offsets['POSITION'] = offset
                elif semantic == 'NORMAL' and 'NORMAL' not in sources:
                    sources['NORMAL'] = inp.attrib['source']
                    offsets['NORMAL'] = offset
            count = int(prim.attrib.get('count', '0'))
            if kind == 'triangles':
                p = self._find(prim, 'p')
                indices = _ints(p.text if p is not None else '')
                vcount = np.full(count, 3, dtype=np.int64)
            elif kind == 'polylist':
                vc = self._find(prim, 'vcount')
                p = self._find(prim, 'p')
                vcount = _ints(vc.text if vc is not None else '')
                indices = _ints(p.text if p is not None else '')
            else:  # polygons: one <p> per polygon (no holes supported)
                chunks = [_ints(p.text) for p in self._findall(prim, 'p')]
                vcount = np.array([len(c) // stride for c in chunks], dtype=np.int64)
                indices = np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.int64)
            out.append((prim.attrib.get('material', ''), sources, vcount, indices, offsets, stride))
        return out

    def _geometry_submeshes(self, geometry, transform: np.ndarray,
                            bindings: Dict[str, str]) -> List[SubMesh]:
        mesh = self._find(geometry, 'mesh')
        if mesh is None:
            return []
        rot = transform[:3, :3]
        normal_rot = np.linalg.inv(rot).T if abs(np.linalg.det(rot)) > 1e-12 else rot
        submeshes: List[SubMesh] = []
        for symbol, sources, vcount, indices, offsets, stride in self._primitives(mesh):
            if 'POSITION' not in sources or len(indices) == 0:
                continue
            positions = self._source_array(sources['POSITION']) * self.unit
            positions = positions @ rot.T + transform[:3, 3] * self.unit
            normals = None
            if 'NORMAL' in sources:
                normals = self._source_array(sources['NORMAL']) @ normal_rot.T
                lengths = np.linalg.norm(normals, axis=1)
                lengths[lengths == 0] = 1.0
                normals = normals / lengths[:, None]
            corners = indices.reshape(-1, stride)
            pos_idx = corners[:, offsets['POSITION']]
            nrm_idx = corners[:, offsets['NORMAL']] if normals is not None else np.full(len(corners), -1)
            # Fan-triangulate polygons; the corner order is preserved so winding is kept.
            starts = np.concatenate([[0], np.cumsum(vcount)[:-1]])
            tri_corners = []
            for start, n in zip(starts, vcount):
                for k in range(1, n - 1):
                    tri_corners.append((start, start + k, start + k + 1))
            tri = np.array(tri_corners, dtype=np.int64).reshape(-1, 3)
            faces = np.stack([pos_idx[tri], nrm_idx[tri]], axis=-1)
            if np.linalg.det(rot) < 0:  # mirrored transform flips the winding
                faces = faces[:, ::-1, :]
            material_id = bindings.get(symbol, symbol)
            submeshes.append(SubMesh(
                material=self.material_key(material_id),
                rgba=self.material_rgba(material_id),
                positions=positions,
                normals=normals,
                faces=faces,
            ))
        return submeshes

    def _visit(self, node, parent: np.ndarray, out: List[SubMesh]) -> None:
        transform = parent @ _node_matrix(node, self.ns)
        for inst in self._findall(node, 'instance_geometry'):
            geometry = self._ref(inst.attrib.get('url', ''))
            if geometry is None:
                continue
            bindings = {
                im.attrib['symbol']: im.attrib.get('target', '').lstrip('#')
                for im in inst.iter(self._tag('instance_material')) if 'symbol' in im.attrib
            }
            out.extend(self._geometry_submeshes(geometry, transform, bindings))
        for inst in self._findall(node, 'instance_node'):
            target = self._ref(inst.attrib.get('url', ''))
            if target is not None:
                self._visit(target, transform, out)
        for child in self._findall(node, 'node'):
            self._visit(child, transform, out)

    def submeshes(self) -> List[SubMesh]:
        out: List[SubMesh] = []
        scene = self._find(self.root, 'scene')
        scenes = []
        if scene is not None:
            for inst in self._findall(scene, 'instance_visual_scene'):
                target = self._ref(inst.attrib.get('url', ''))
                if target is not None:
                    scenes.append(target)
        if not scenes:
            scenes = list(self.root.iter(self._tag('visual_scene')))
        y_up = np.eye(4)
        if self.up_axis == 'Y_UP':
            y_up[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
        elif self.up_axis == 'X_UP':
            y_up[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        for vs in scenes:
            for node in self._findall(vs, 'node'):
                self._visit(node, y_up, out)
        if not out:  # no scene: fall back to the raw geometries
            for geometry in self.root.iter(self._tag('geometry')):
                out.extend(self._geometry_submeshes(geometry, y_up, {}))
        return merge_by_material(out)


def merge_by_material(parts: Sequence[SubMesh]) -> List[SubMesh]:
    """Concatenate submeshes that share a material (one OBJ per colour)."""
    merged: Dict[str, SubMesh] = {}
    order: List[str] = []
    for part in parts:
        key = part.material
        if key not in merged:
            merged[key] = SubMesh(part.material, part.rgba, part.positions, part.normals, part.faces)
            order.append(key)
            continue
        cur = merged[key]
        faces = part.faces.copy()
        faces[:, :, 0] += len(cur.positions)
        if cur.normals is not None and part.normals is not None:
            faces[:, :, 1] += len(cur.normals)
            normals = np.vstack([cur.normals, part.normals])
        else:
            faces[:, :, 1] = -1
            cur.faces = cur.faces.copy()
            cur.faces[:, :, 1] = -1
            normals = None
        merged[key] = SubMesh(cur.material, cur.rgba, np.vstack([cur.positions, part.positions]),
                              normals, np.vstack([cur.faces, faces]))
    return [merged[k] for k in order]


# ---------------------------------------------------------------------------
# STL
# ---------------------------------------------------------------------------

def load_stl(path: Path) -> List[SubMesh]:
    data = path.read_bytes()
    triangles: Optional[np.ndarray] = None
    if len(data) >= 84:
        count = struct.unpack('<I', data[80:84])[0]
        if 84 + 50 * count == len(data):
            records = np.frombuffer(data[84:84 + 50 * count], dtype=np.dtype([
                ('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')]))
            triangles = records['v'].astype(np.float64)
    if triangles is None:
        values = re.findall(r'vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)', data.decode('ascii', 'ignore'))
        triangles = np.array(values, dtype=np.float64).reshape(-1, 3, 3)
    positions = triangles.reshape(-1, 3)
    index = np.arange(len(positions), dtype=np.int64).reshape(-1, 3)
    faces = np.stack([index, np.full_like(index, -1)], axis=-1)
    return [SubMesh(material='', rgba=DEFAULT_RGBA, positions=positions, normals=None, faces=faces)]


# ---------------------------------------------------------------------------
# OBJ output
# ---------------------------------------------------------------------------

def write_obj(path: Path, part: SubMesh, source_name: str) -> None:
    lines = [f'# {source_name} material {part.material or "-"}; written by tools/champ_mesh_assets.py',
             f'# rgba {" ".join(f"{v:g}" for v in part.rgba)}']
    used_pos = np.unique(part.faces[:, :, 0])
    pos_remap = np.full(len(part.positions), -1, dtype=np.int64)
    pos_remap[used_pos] = np.arange(1, len(used_pos) + 1)
    for p in part.positions[used_pos]:
        lines.append(f'v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}')
    has_normals = part.normals is not None and bool(np.all(part.faces[:, :, 1] >= 0))
    if has_normals:
        used_nrm = np.unique(part.faces[:, :, 1])
        nrm_remap = np.full(len(part.normals), -1, dtype=np.int64)
        nrm_remap[used_nrm] = np.arange(1, len(used_nrm) + 1)
        for n in part.normals[used_nrm]:
            lines.append(f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}')
        for f in part.faces:
            a, b, c = pos_remap[f[:, 0]]
            na, nb, nc = nrm_remap[f[:, 1]]
            lines.append(f'f {a}//{na} {b}//{nb} {c}//{nc}')
    else:
        for f in part.faces:
            a, b, c = pos_remap[f[:, 0]]
            lines.append(f'f {a} {b} {c}')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


# ---------------------------------------------------------------------------
# Conversion driver
# ---------------------------------------------------------------------------

def load_mesh_file(path: Path) -> List[SubMesh]:
    suffix = path.suffix.lower()
    if suffix == '.dae':
        return ColladaFile(path).submeshes()
    if suffix == '.stl':
        return load_stl(path)
    raise ValueError(f'{path}: unsupported visual mesh format {suffix!r} (dae/stl)')


def _sha1(path: Path) -> str:
    h = hashlib.sha1()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def assets_dir(package_dir: Path, robot: str) -> Path:
    return Path(package_dir) / 'mujoco' / 'assets' / robot


def _load_manifest(directory: Path) -> Dict:
    path = directory / MANIFEST_NAME
    if not path.exists():
        return {'version': MANIFEST_VERSION, 'sources': {}}
    try:
        manifest = json.loads(path.read_text(encoding='utf-8'))
    except json.JSONDecodeError:
        return {'version': MANIFEST_VERSION, 'sources': {}}
    if manifest.get('version') != MANIFEST_VERSION:
        return {'version': MANIFEST_VERSION, 'sources': {}}
    manifest.setdefault('sources', {})
    return manifest


def urdf_visual_mesh_files(urdf: Urdf) -> List[str]:
    files: List[str] = []
    for link in urdf.links.values():
        for visual in link.visuals:
            if visual.geometry == 'mesh' and visual.attrib.get('filename') and visual.attrib['filename'] not in files:
                files.append(visual.attrib['filename'])
    return files


def convert_robot_meshes(package_dir: Path, robot: str, urdf: Urdf, force: bool = False,
                         log=print) -> MeshAssets:
    """Convert every URDF visual mesh of `robot`; return the asset table."""
    package_dir = Path(package_dir)
    directory = assets_dir(package_dir, robot)
    directory.mkdir(parents=True, exist_ok=True)
    manifest = _load_manifest(directory)
    sources = manifest['sources']
    wanted = urdf_visual_mesh_files(urdf)
    assets = MeshAssets(directory=directory)
    keep_files = {MANIFEST_NAME}
    for filename in wanted:
        source = resolve_mesh_path(filename, package_dir)
        if not source.exists():
            raise FileNotFoundError(f'visual mesh {filename} -> {source} not found')
        digest = _sha1(source)
        stem = mesh_asset_stem(source, package_dir)
        cached = sources.get(filename)
        cached_ok = (
            not force and cached is not None and cached.get('sha1') == digest
            and all((directory / p['file']).exists() for p in cached.get('parts', []))
        )
        if cached_ok:
            parts = cached['parts']
        else:
            submeshes = load_mesh_file(source)
            parts = []
            for index, part in enumerate(submeshes):
                if len(part.faces) == 0:
                    continue
                name = f'{stem}_{index}'
                out = directory / f'{name}.obj'
                write_obj(out, part, source.name)
                parts.append({
                    'name': name, 'file': out.name, 'material': part.material,
                    'rgba': [round(float(v), 7) for v in part.rgba], 'triangles': int(len(part.faces)),
                })
            sources[filename] = {'sha1': digest, 'source': str(source.relative_to(package_dir))
                                 if source.is_relative_to(package_dir) else str(source), 'parts': parts}
            log(f'converted {source.relative_to(package_dir) if source.is_relative_to(package_dir) else source}'
                f' -> {len(parts)} obj ({sum(p["triangles"] for p in parts)} triangles)')
        assets.entries[filename] = [AssetEntry(**p) for p in parts]
        keep_files.update(p['file'] for p in parts)
    # Drop manifest entries and files for meshes the URDF no longer references.
    for filename in list(sources):
        if filename not in wanted:
            del sources[filename]
    for path in directory.glob('*.obj'):
        if path.name not in keep_files:
            path.unlink()
    manifest['robot'] = robot
    (directory / MANIFEST_NAME).write_text(json.dumps(manifest, indent=1, ensure_ascii=False) + '\n',
                                           encoding='utf-8')
    return assets


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('robot')
    parser.add_argument('--package-dir', type=Path, default=SOURCE_PACKAGE_DIR)
    parser.add_argument('--force', action='store_true', help='reconvert even when the manifest is current')
    args = parser.parse_args()
    files = resolve_robot_files(args.package_dir, args.robot)
    urdf = load_urdf(files.urdf)
    assets = convert_robot_meshes(args.package_dir, args.robot, urdf, force=args.force)
    total = sum(len(parts) for parts in assets.entries.values())
    print(f'{args.robot}: {len(assets.entries)} visual meshes -> {total} obj files in {assets.directory}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
