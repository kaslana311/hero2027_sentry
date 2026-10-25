#!/usr/bin/env python3
"""Prepare a CAD-derived 3-D localization prior, WITHOUT reading/changing a PGM.

Only the reviewed simulator mesh is accepted. Synthetic keyframe scans are not
real-robot mapping data; global relocalization still needs field validation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

MESH_SHA = 'd5916a8ab5e3d1fcacfc2cd4cca0497ad5d73947ad2b763408fb459f0e78bf97'
MESH_NAME = 'rmuc_2027_tunnel_lips_repaired.stl'
MODEL_TRANSLATION = (-0.14910008013248444, -0.06378698348999023, -0.27757187763365515)
SPAWN_XY = (4.6, 6.4)
SENSOR_HEIGHT = 0.197  # chassis .076 + virtual IMU .121, metres above support
VOXEL = 0.05


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_triangles(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError('Truncated binary STL')
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + count*50:
        raise ValueError('Unexpected binary STL length')
    dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attribute', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, offset=84)['v'].astype(np.float64)
    if not np.isfinite(triangles).all():
        raise ValueError('Nonfinite STL geometry')
    return triangles


def voxelize(points, leaf):
    if not math.isfinite(leaf) or leaf <= 0:
        raise ValueError('voxel size must be positive')
    _, keep = np.unique(np.floor(points / leaf).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(keep)]


def sample_surface(triangles, leaf=VOXEL):
    # Area-weighted deterministic barycentric samples, plus ALL original vertices.
    # No flattening, ground removal, hole filling or occupancy-column filtering.
    rng = np.random.default_rng(20271025)
    pieces = [triangles.reshape(-1, 3)]
    for begin in range(0, len(triangles), 8192):
        tri = triangles[begin:begin+8192]
        areas = 0.5*np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)
        counts = np.ceil(areas / (leaf*leaf/4)).astype(np.int64)
        ids = np.repeat(np.arange(len(tri)), counts)
        uv = rng.random((len(ids), 2))
        folded = uv.sum(axis=1) > 1
        uv[folded] = 1-uv[folded]
        pieces.append(tri[ids, 0] + uv[:, :1]*(tri[ids, 1]-tri[ids, 0]) +
                      uv[:, 1:]*(tri[ids, 2]-tri[ids, 0]))
    return voxelize(np.concatenate(pieces), leaf)


def write_pcd(path, points):
    data = np.empty((len(points), 4), dtype='<f4')
    data[:, :3], data[:, 3] = points, 1.0
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
              'TYPE F F F F\nCOUNT 1 1 1 1\n' + f'WIDTH {len(points)}\nHEIGHT 1\n' +
              'VIEWPOINT 0 0 0 1 0 0 0\n' + f'POINTS {len(points)}\nDATA binary\n')
    with path.open('xb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(data.tobytes())


def support_height(triangles, bounds, x, y):
    mask = ((bounds[0][:, 0] <= x) & (x <= bounds[1][:, 0]) &
            (bounds[0][:, 1] <= y) & (y <= bounds[1][:, 1]))
    tri = triangles[mask]
    a, b = tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]
    determinant = a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]
    valid = np.abs(determinant) > 1e-12
    tri, a, b, determinant = tri[valid], a[valid], b[valid], determinant[valid]
    dx, dy = x-tri[:, 0, 0], y-tri[:, 0, 1]
    u = (dx*b[:, 1]-dy*b[:, 0])/determinant
    v = (a[:, 0]*dy-a[:, 1]*dx)/determinant
    inside = (u >= -1e-7) & (v >= -1e-7) & (u+v <= 1+1e-7)
    heights = tri[:, 0, 2]+u*a[:, 2]+v*b[:, 2]
    normals = np.cross(a, b)
    walkable = np.abs(normals[:, 2])/np.maximum(np.linalg.norm(normals, axis=1), 1e-20) >= math.cos(math.radians(20))
    all_z = np.sort(heights[inside])
    # Reject the foundation underside. Physical floor spans about [-.168,+.252] m.
    candidates = np.sort(heights[inside & walkable & (heights >= -.20) & (heights <= .85)])
    for height in candidates:
        above = all_z[all_z > height + .005]
        if not len(above) or above[0]-height >= .24:
            return float(height)
    return None


def synthetic_scan(points, origin):
    local = points-origin
    distance2 = (local*local).sum(axis=1)
    keep = (distance2 >= .3**2) & (distance2 <= 12.0**2)
    local, distance2 = local[keep], distance2[keep]
    elevation = np.arctan2(local[:, 2], np.linalg.norm(local[:, :2], axis=1))
    keep = np.abs(elevation) < math.radians(85)
    local, distance2, elevation = local[keep], distance2[keep], elevation[keep]
    # Angular-bin nearest surfaces approximate visibility, not a Gazebo sensor replay.
    azimuth = np.arctan2(local[:, 1], local[:, 0])
    columns = np.minimum(1439, ((azimuth+math.pi)/(2*math.pi)*1440).astype(int))
    rows = np.minimum(179, ((elevation+math.pi/2)/math.pi*180).astype(int))
    order = np.argsort(distance2, kind='stable')
    _, first = np.unique((rows*1440+columns)[order], return_index=True)
    return voxelize(local[order[first]], .1)


def prepare(sim_repo, output, descriptor_tool):
    sim_repo, output = Path(sim_repo).resolve(), Path(output).resolve()
    model = sim_repo / 'overlays/rmu_gazebo_simulator/resource/models/rmuc_2027'
    mesh = model / 'meshes' / MESH_NAME
    if digest(mesh) != MESH_SHA:
        raise ValueError('Reviewed mesh hash mismatch; do not silently reuse an obsolete prior')
    sdf = ET.parse(model / 'model.sdf').getroot()
    pose = tuple(float(v) for v in sdf.find('./model/link/pose').text.split())
    if len(pose) != 6 or not np.allclose(pose[:3], MODEL_TRANSLATION, atol=1e-12, rtol=0) or any(pose[3:]):
        raise ValueError('Field placement changed; review the prior coordinate calibration')
    uri = sdf.find('./model/link/collision/geometry/mesh/uri').text
    if not uri.endswith('/'+MESH_NAME):
        raise ValueError('The running field selects a different collision mesh')
    if output.exists():
        raise ValueError(f'Refusing to overwrite {output}; choose a new versioned output directory')
    output.parent.mkdir(parents=True, exist_ok=True)
    staged = Path(tempfile.mkdtemp(prefix=output.name+'.building-', dir=output.parent))
    print(f'New map staging directory: {staged}', flush=True)
    offset = np.array(MODEL_TRANSLATION)-np.array((*SPAWN_XY, 0.0))
    triangles = read_triangles(mesh)+offset
    points = sample_surface(triangles)
    write_pcd(staged/'map.pcd', points)
    bounds = (triangles.min(axis=1), triangles.max(axis=1))
    (staged/'clouds').mkdir()
    positions = [(0.0, 0.0)] + [(float(x), float(y)) for y in np.arange(-5.9, 9.2, 2.)
                                for x in np.arange(-4.1, 23., 2.)]
    rows, supports = [], []
    for x, y in positions:
        height = support_height(triangles, bounds, x, y)
        if height is None:
            continue
        origin = np.array((x, y, height+SENSOR_HEIGHT))
        cloud = synthetic_scan(points, origin)
        if len(cloud) < 100:
            continue
        index = len(rows)
        write_pcd(staged/'clouds'/f'{index:06d}.pcd', cloud)
        rows.append(f'{index} {1000+index*.5:.17g} {x:.17g} {y:.17g} {origin[2]:.17g} 0 0 0 1\n')
        supports.append([x, y, height, len(cloud)])
    if len(rows) < 30:
        raise ValueError(f'Insufficient prior coverage: only {len(rows)} synthetic keyframes')
    (staged/'keyframes.txt').write_text('# backend_map 1\n# index timestamp tx ty tz qx qy qz qw\n'+''.join(rows))
    subprocess.run([str(descriptor_tool), '--bundle', str(staged), '--num-rings', '30',
                    '--num-sectors', '90', '--max-radius', '20', '--downsample', '.1',
                    '--lidar-height', str(SENSOR_HEIGHT), '--level', 'true',
                    '--augment-count', '3', '--lateral-shift', '2', '--search-ratio', '1'], check=True)
    manifest = {'schema': 1, 'purpose': 'EXPERIMENTAL simulation localization prior; not navigation PGM',
                'frame_id': 'map', 'mesh': str(mesh), 'mesh_sha256': MESH_SHA,
                'model_translation': MODEL_TRANSLATION, 'nominal_spawn_world_xy': SPAWN_XY,
                'raw_stl_to_map_translation': offset.tolist(), 'global_surface_voxel_m': VOXEL,
                'points': len(points), 'synthetic_keyframes': len(rows), 'supports_xy_z_pointcount': supports,
                'limitations': ['CAD synthetic scans, not real recorded keyframes',
                                'Symmetric-field global recovery is NOT certified',
                                'Nominal spawn XY; truth observer origin can differ by settling',
                                'No live truth used to generate or update localization'],
                'files_sha256': {str(p.relative_to(staged)): digest(p) for p in sorted(staged.rglob('*')) if p.is_file()}}
    (staged/'hero_prior_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    staged.rename(output)
    print(f'Prepared {output}: {len(points)} surface points, {len(rows)} keyframes. PGM unchanged.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-repo', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--descriptor-tool', help='Optional explicit pinned backend tool path')
    args = parser.parse_args()
    if args.descriptor_tool:
        tool = Path(args.descriptor_tool).resolve()
    else:
        from ament_index_python.packages import get_package_prefix
        tool = Path(get_package_prefix('backend'))/'lib/backend/rebuild_scan_context_descriptors'
    if not tool.is_file():
        parser.error('Compile the backend with build_hero2026_safe.sh before preparing a map')
    prepare(args.sim_repo, args.output_dir, tool)


if __name__ == '__main__':
    main()
