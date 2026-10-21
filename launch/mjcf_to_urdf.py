#!/usr/bin/env python3
"""Print a URDF robot_description, converting a MuJoCo MJCF if needed.

A URDF is printed unchanged. An MJCF is compiled with mujoco (so defaults,
childclass, fromto and angle units are already resolved) and rewritten as a
URDF: one link per body, one link per named site, and the collision geoms
(contype or conaffinity != 0) as <collision> elements.

robot_self_filter only knows spheres, boxes, cylinders and meshes, so a
capsule becomes a cylinder plus one sphere per end cap (exact, before padding).
Collision meshes are skipped: MuJoCo recenters mesh vertices, so the original
file cannot be reused with the compiled geom pose.

Usage: mjcf_to_urdf.py <model.xml|model.urdf> [--root-link base_link]
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def fmt(values):
    return ' '.join(f'{v + 0.0:.9g}' for v in values)  # + 0.0 turns -0 into 0


def quat_to_mat(quat):
    w, x, y, z = quat / np.linalg.norm(quat)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_rpy(quat):
    """MuJoCo quaternion (w, x, y, z) -> URDF fixed-axis roll, pitch, yaw."""
    r = quat_to_mat(quat)
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if math.cos(pitch) > 1e-9:
        return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])
    # Gimbal lock: only roll - yaw (pitch = +90) or roll + yaw (pitch = -90)
    # is defined, so put it all in roll.
    return math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0


def add_origin(parent, pos, quat):
    ET.SubElement(parent, 'origin', xyz=fmt(pos), rpy=fmt(quat_to_rpy(quat)))


def add_collision(link, pos, quat, geometry, **size):
    collision = ET.SubElement(link, 'collision')
    add_origin(collision, pos, quat)
    ET.SubElement(ET.SubElement(collision, 'geometry'), geometry,
                  {k: fmt(v) for k, v in size.items()})


def warn(message):
    print(f'mjcf_to_urdf: {message}', file=sys.stderr)


def add_joint(robot, model, mujoco, names, b):
    """Joint from the parent link to body b, fixed unless URDF can express it."""
    name = names[b]
    joint_ids = range(model.body_jntadr[b], model.body_jntadr[b] + model.body_jntnum[b])
    joint_type = 'fixed'
    if len(joint_ids) == 1 and not np.any(model.jnt_pos[joint_ids[0]]):
        kind = model.jnt_type[joint_ids[0]]
        if kind == mujoco.mjtJoint.mjJNT_HINGE:
            joint_type = 'revolute' if model.jnt_limited[joint_ids[0]] else 'continuous'
        elif kind == mujoco.mjtJoint.mjJNT_SLIDE:
            joint_type = 'prismatic'
    if joint_type == 'fixed' and len(joint_ids) > 0:
        warn(f'the joints of body {name} have no URDF equivalent, exporting them as fixed')

    joint_name = model.joint(joint_ids[0]).name if joint_type != 'fixed' else ''
    joint = ET.SubElement(robot, 'joint', name=joint_name or f'{name}_joint', type=joint_type)
    ET.SubElement(joint, 'parent', link=names[model.body_parentid[b]])
    ET.SubElement(joint, 'child', link=name)
    add_origin(joint, model.body_pos[b], model.body_quat[b])
    if joint_type != 'fixed':
        j = joint_ids[0]
        ET.SubElement(joint, 'axis', xyz=fmt(model.jnt_axis[j]))
        if joint_type != 'continuous':
            # effort/velocity are mandatory in URDF but unused by the self filter.
            ET.SubElement(joint, 'limit', lower=fmt([model.jnt_range[j][0]]),
                          upper=fmt([model.jnt_range[j][1]]), effort='0', velocity='0')


def mjcf_to_urdf(path, robot_name, root_link):
    import mujoco

    model = mujoco.MjModel.from_xml_path(path)
    roots = [b for b in range(1, model.nbody) if model.body_parentid[b] == 0]
    if len(roots) != 1:
        sys.exit(f'mjcf_to_urdf: {path} must have exactly one root body, found {len(roots)}')
    names = {b: model.body(b).name or f'body_{b}' for b in range(1, model.nbody)}
    if root_link:
        names[roots[0]] = root_link

    robot = ET.Element('robot', name=robot_name)
    for b, name in names.items():
        link = ET.SubElement(robot, 'link', name=name)
        for g in np.flatnonzero(model.geom_bodyid == b):
            if not (model.geom_contype[g] or model.geom_conaffinity[g]):
                continue
            pos, quat, size = model.geom_pos[g], model.geom_quat[g], model.geom_size[g]
            gtype = model.geom_type[g]
            if gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
                add_collision(link, pos, quat, 'sphere', radius=[size[0]])
            elif gtype == mujoco.mjtGeom.mjGEOM_BOX:
                add_collision(link, pos, quat, 'box', size=2 * size)
            elif gtype == mujoco.mjtGeom.mjGEOM_CYLINDER:
                add_collision(link, pos, quat, 'cylinder', radius=[size[0]], length=[2 * size[1]])
            elif gtype == mujoco.mjtGeom.mjGEOM_CAPSULE:
                add_collision(link, pos, quat, 'cylinder', radius=[size[0]], length=[2 * size[1]])
                half_axis = size[1] * quat_to_mat(quat)[:, 2]
                for end in (pos + half_axis, pos - half_axis):
                    add_collision(link, end, quat, 'sphere', radius=[size[0]])
            else:
                warn(f'skipping {mujoco.mjtGeom(gtype).name} collision geom {g} on body {name}')

    for b in names:
        if b not in roots:
            add_joint(robot, model, mujoco, names, b)

    for s in range(model.nsite):
        b = model.site_bodyid[s]
        name = model.site(s).name
        if b not in names or not name or name in names.values():
            continue
        ET.SubElement(robot, 'link', name=name)
        joint = ET.SubElement(robot, 'joint', name=f'{name}_joint', type='fixed')
        ET.SubElement(joint, 'parent', link=names[b])
        ET.SubElement(joint, 'child', link=name)
        add_origin(joint, model.site_pos[s], model.site_quat[s])

    ET.indent(robot)
    return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding='unicode') + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('model', help='URDF or MuJoCo MJCF file')
    parser.add_argument('--root-link', default='',
                        help='URDF name of the MJCF root body (default: the body name)')
    args = parser.parse_args()

    root = ET.parse(args.model).getroot()
    if root.tag == 'robot':
        with open(args.model) as urdf:
            sys.stdout.write(urdf.read())
    else:
        sys.stdout.write(mjcf_to_urdf(args.model, root.get('model', 'robot'), args.root_link))


if __name__ == '__main__':
    main()
