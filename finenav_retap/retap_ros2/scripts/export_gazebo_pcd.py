#!/usr/bin/env python3
"""Sample a Gazebo model's terrain mesh into a PCD, in world coordinates.

The S5 demo needs one map with one geometry: the terrain mesh Gazebo actually
simulates, the prior .bt the planner renders, and the live lidar scans must all
be the same surface.  `finenav_simulator/resource/maps/*.pcd` are NOT that
surface for the desert world -- they were exported from the mesh without the
SDF's own `<scale>`, so the world is a different size and sits at a different
height.  Re-sampling the mesh here, using the SDF, keeps the three in step.

    ros2 run finenav_retap export_gazebo_pcd \
        --model <finenav_simulator>/resource/model/map_desert \
        --output /tmp/retap_gazebo/map_desert_world.pcd --spacing 0.045

How the world transform is built (both parts matter):

    world = sdf_scale * (collada_matrix * vertex)

gzsdf applies the mesh's own <scale>, and the DAE applies its node matrix;
SDF mesh scaling is applied last, to the COLLADA-transformed geometry.  The
desert mesh is +-1 m in the DAE, with a node matrix that scales x,y by 6 and z
by 1, and an SDF scale of 2: the world is therefore 24 m x 24 m, z 0.07..3.98.

Normals use the inverse-transpose of that (anisotropic) transform.  The
sampling density is points per square metre: keep it at the density the
existing 0.2 m terrain calibration expects (the simulator's own desert.pcd has
about 690 points/m^2, i.e. ~28 points per 0.2 m voxel).

`--offset-z` exists because of a second, unrelated shift: the simulator's
odometry (and therefore the whole TF tree, and therefore `map`) reports
`base_origin`'s pose as `base_link`, while the robot SDF puts base_link 0.75 m
below base_origin (`base_origin_to_base_link`).  The live scans are handed to
the map in that shifted frame, so the exported cloud must be shifted too
(`--offset-z 0.75` for the desert world) or the prior and the scans disagree by
exactly that amount -- the "two surfaces" failure.
"""

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np


def _parse_float_array(text, count, stride):
    values = np.fromstring(text.replace("\n", " "), sep=" ", dtype=np.float64)
    return values.reshape(-1, stride)


def load_mesh(dae_path):
    """Return (vertices, triangles, normals) from a COLLADA file."""
    with open(dae_path, "r", encoding="utf-8", errors="ignore") as handle:
        text = handle.read()

    positions = re.search(
        r'<float_array id="[^"]*positions[^"]*"[^>]*>(.*?)</float_array>',
        text, re.S)
    if positions is None:
        raise SystemExit(f"{dae_path}: no POSITION array")
    vertices = np.array(
        [float(v) for v in positions.group(1).split()], dtype=np.float64)
    vertices = vertices.reshape(-1, 3)

    normals = np.zeros_like(vertices)
    normals_found = re.search(
        r'<float_array id="[^"]*normals[^"]*"[^>]*>(.*?)</float_array>',
        text, re.S)
    if normals_found is not None:
        raw = np.array(
            [float(v) for v in normals_found.group(1).split()], dtype=np.float64)
        if raw.size % 3 == 0:
            normals = raw.reshape(-1, 3)[:len(vertices)]

    # <p> holds, per triangle corner, `stride` indices (vertex, normal, uv...).
    # Read the <input> offsets so the vertex offset does not have to be 0.
    triangles = []
    for mesh_block in re.finditer(
            r"<mesh>(.*?)</mesh>", text, re.S):
        block = mesh_block.group(1)
        vertex_offset = 0
        stride = 1
        # Only inputs that carry an `offset` take part in the corner stride;
        # the <input semantic="POSITION"> describing the array has none.
        inputs = [attributes for attributes in re.findall(r"<input([^>]*)/>", block)
                  if 'offset="' in attributes]
        for attributes in inputs:
            if 'semantic="VERTEX"' in attributes:
                vertex_offset = int(
                    re.search(r'offset="(\d+)"', attributes).group(1))
        for attributes in inputs:
            offset = int(re.search(r'offset="(\d+)"', attributes).group(1))
            stride = max(stride, offset + 1)
        for p_block in re.findall(r"<p>(.*?)</p>", block, re.S):
            indices = np.array(
                [int(v) for v in p_block.split()], dtype=np.int64)
            indices = indices.reshape(-1, stride)[:, vertex_offset]
            triangles.append(indices.reshape(-1, 3))
    if not triangles:
        raise SystemExit(f"{dae_path}: no <triangles>")
    return vertices, np.concatenate(triangles), normals


def load_transforms(model_dir):
    """Return the world transform: SDF mesh <scale> * COLLADA node matrix."""
    sdf_path = os.path.join(model_dir, "model-1_4.sdf")
    if not os.path.exists(sdf_path):
        candidates = sorted(
            name for name in os.listdir(model_dir) if name.endswith(".sdf"))
        if not candidates:
            raise SystemExit(f"{model_dir}: no .sdf found")
        sdf_path = os.path.join(model_dir, candidates[0])

    root = ET.parse(sdf_path).getroot()
    scales = root.findall(".//mesh/scale")
    sdf_scale = np.ones(3)
    if scales:
        sdf_scale = np.array([float(v) for v in scales[0].text.split()])
    dae = root.find(".//mesh/uri").text
    # model://<model>/meshes/<file>.dae -> <model_dir>/meshes/<file>.dae
    relative = dae.split("/", 3)[-1]
    dae_path = os.path.join(model_dir, relative)

    matrix = np.eye(4)
    with open(dae_path, "r", encoding="utf-8", errors="ignore") as handle:
        match = re.search(
            r'<matrix sid="transform">(.*?)</matrix>', handle.read(), re.S)
    if match is not None:
        matrix = np.array([float(v) for v in match.group(1).split()],
                          dtype=np.float64).reshape(4, 4).T
    world = np.diag(np.append(sdf_scale, 1.0)) @ matrix
    return dae_path, world


def sample(vertices, triangles, normals, world, spacing):
    homogeneous = np.hstack([vertices, np.ones((len(vertices), 1))])
    vertices = (world @ homogeneous.T).T[:, :3]
    # Anisotropic scaling needs the inverse transpose for directions.
    normal_matrix = np.linalg.inv(world[:3, :3]).T
    if normals.any():
        normals = normals @ normal_matrix.T
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = np.divide(normals, lengths, out=np.zeros_like(normals),
                            where=lengths > 1e-9)

    a = vertices[triangles[:, 0]]
    b = vertices[triangles[:, 1]]
    c = vertices[triangles[:, 2]]
    face_area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    counts = np.maximum((face_area / (spacing * spacing)).round().astype(np.int64), 1)
    total = int(counts.sum())

    rng = np.random.default_rng(20261023)
    triangle_ids = np.repeat(np.arange(len(triangles)), counts)
    # Uniform barycentric sampling.
    u = rng.random(total)
    v = rng.random(total)
    flip = u + v > 1.0
    u[flip] = 1.0 - u[flip]
    v[flip] = 1.0 - v[flip]
    points = (a[triangle_ids] +
              u[:, None] * (b[triangle_ids] - a[triangle_ids]) +
              v[:, None] * (c[triangle_ids] - a[triangle_ids]))
    point_normals = normals[triangles[triangle_ids, 0]]

    return points, point_normals, float(face_area.sum())


def write_pcd(path, points, normals):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    data = np.hstack([points, normals]).astype(np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z normal_x normal_y normal_z\n"
        "SIZE 4 4 4 4 4 4\n"
        "TYPE F F F F F F\n"
        "COUNT 1 1 1 1 1 1\n"
        f"WIDTH {len(data)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(data)}\n"
        "DATA binary\n")
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(data.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True,
                        help="Gazebo model directory (contains model-1_4.sdf)")
    parser.add_argument("--output", required=True, help="output .pcd")
    parser.add_argument("--spacing", type=float, default=0.045,
                        help="sampling distance in metres (default 0.045)")
    parser.add_argument("--offset-z", type=float, default=0.0,
                        help="z shift applied after the model transform; the "
                             "desert world needs 0.75 (see the S5 notes)")
    args = parser.parse_args()

    dae_path, world = load_transforms(args.model)
    vertices, triangles, normals = load_mesh(dae_path)
    points, point_normals, area = sample(
        vertices, triangles, normals, world, args.spacing)
    points[:, 2] += args.offset_z
    write_pcd(args.output, points, point_normals)

    bounds = np.vstack([points.min(axis=0), points.max(axis=0)])
    print(f"{os.path.basename(args.model)}: {len(points)} points over "
          f"{area:.1f} m2; x [{bounds[0,0]:.2f}, {bounds[1,0]:.2f}] "
          f"y [{bounds[0,1]:.2f}, {bounds[1,1]:.2f}] "
          f"z [{bounds[0,2]:.2f}, {bounds[1,2]:.2f}]")
    print(f"  world transform (SDF scale x COLLADA matrix):\n{world}")
    print(f"  wrote {args.output}")

    points_per_m2 = len(points) / max(area, 1e-9)
    print(f"  density {points_per_m2:.0f} pts/m2 "
          f"(~{points_per_m2 * 0.04:.0f} pts per 0.2 m voxel)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
