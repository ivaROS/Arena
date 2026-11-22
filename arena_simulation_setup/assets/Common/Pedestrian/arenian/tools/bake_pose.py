#!/usr/bin/env python3
# Source mesh: https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae
# Regenerate: python3 bake_pose.py walk.dae ../meshes/arenian_walk.dae --frame 17
"""Bake one animation frame of a skinned COLLADA mesh into a static (non-skinned) mesh.

Gazebo draws a skinned mesh that is used as a plain <visual> in its bind pose. For the
MakeHuman/CMU rig of the Fuel "actor" model that bind pose is a T-pose. This script
applies linear-blend skinning at one keyframe and writes a DAE with only geometry and
materials, so a static <model> pedestrian shows a natural pose.

    python3 bake_pose.py walk.dae out.dae --frame 0 [--keep-root-xy]

Hip translation in the horizontal plane is removed by default so the baked mesh stays
centred on the model origin (walk clips move the hips forward over time).
"""

import argparse
import copy
import xml.etree.ElementTree as ET

import numpy as np

NS = 'http://www.collada.org/2005/11/COLLADASchema'
C = '{%s}' % NS
ET.register_namespace('', NS)


def floats(el):
    return np.array(el.text.split(), dtype=float)


def node_local_matrix(node):
    """Compose a <node>'s transform elements in document order."""
    m = np.eye(4)
    for t in node:
        tag = t.tag.replace(C, '')
        if tag == 'matrix':
            m = m @ floats(t).reshape(4, 4)
        elif tag == 'translate':
            x = np.eye(4)
            x[:3, 3] = floats(t)
            m = m @ x
        elif tag == 'rotate':
            ax, ay, az, ang = floats(t)
            a = np.array([ax, ay, az]) / np.linalg.norm([ax, ay, az])
            th = np.radians(ang)
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
            x = np.eye(4)
            x[:3, :3] = R
            m = m @ x
        elif tag == 'scale':
            m = m @ np.diag([*floats(t), 1.0])
    return m


def source_by_id(root, sid):
    return root.find(f".//{C}source[@id='{sid.lstrip('#')}']")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--frame', type=int, default=0, help='keyframe index to bake')
    ap.add_argument('--keep-root-xy', action='store_true')
    args = ap.parse_args()

    tree = ET.parse(args.src)
    root = tree.getroot()

    # ---- animation: joint id -> local matrix at the chosen key
    anim_local = {}
    for anim in root.iter(f'{C}animation'):
        ch = anim.find(f'{C}channel')
        if ch is None:
            continue
        target = ch.get('target').split('/')[0]
        sampler = anim.find(f"{C}sampler[@id='{ch.get('source').lstrip('#')}']")
        out_src = sampler.find(f"{C}input[@semantic='OUTPUT']").get('source')
        mats = floats(source_by_id(anim, out_src).find(f'{C}float_array')).reshape(-1, 4, 4)
        anim_local[target] = mats[min(args.frame, len(mats) - 1)]

    # ---- skeleton: joint sid -> world matrix (posed)
    scene = root.find(f'{C}library_visual_scenes/{C}visual_scene')
    joint_world = {}
    root_joint = None

    def visit(node, parent_world):
        nonlocal root_joint
        local = node_local_matrix(node)
        if node.get('type') == 'JOINT':
            if node.get('id') in anim_local:
                local = anim_local[node.get('id')]
            if root_joint is None:
                root_joint = node.get('sid')
        world = parent_world @ local
        if node.get('type') == 'JOINT':
            joint_world[node.get('sid')] = world
        for k in node.findall(f'{C}node'):
            visit(k, world)

    for n in scene.findall(f'{C}node'):
        visit(n, np.eye(4))

    if not args.keep_root_xy:
        # cancel horizontal travel of the root joint (in world frame, Z up)
        off = np.eye(4)
        off[:2, 3] = -joint_world[root_joint][:2, 3]
        joint_world = {k: off @ v for k, v in joint_world.items()}

    # ---- skin
    skin = root.find(f'.//{C}skin')
    bsm = floats(skin.find(f'{C}bind_shape_matrix')).reshape(4, 4)
    jnt_input = skin.find(f"{C}joints/{C}input[@semantic='JOINT']").get('source')
    ibm_input = skin.find(f"{C}joints/{C}input[@semantic='INV_BIND_MATRIX']").get('source')
    joint_names = source_by_id(skin, jnt_input).find(f'{C}Name_array').text.split()
    ibms = floats(source_by_id(skin, ibm_input).find(f'{C}float_array')).reshape(-1, 4, 4)
    skin_mats = np.stack([joint_world[j] @ ibms[i] @ bsm for i, j in enumerate(joint_names)])

    vw = skin.find(f'{C}vertex_weights')
    w_src = vw.find(f"{C}input[@semantic='WEIGHT']").get('source')
    weights = floats(source_by_id(skin, w_src).find(f'{C}float_array'))
    j_off = int(vw.find(f"{C}input[@semantic='JOINT']").get('offset'))
    w_off = int(vw.find(f"{C}input[@semantic='WEIGHT']").get('offset'))
    stride = len(vw.findall(f'{C}input'))
    vcount = np.array(vw.find(f'{C}vcount').text.split(), dtype=int)
    v = np.array(vw.find(f'{C}v').text.split(), dtype=int).reshape(-1, stride)

    nverts = len(vcount)
    vert_mat = np.zeros((nverts, 4, 4))
    pos = 0
    for i, c in enumerate(vcount):
        for jj, wi in v[pos:pos + c][:, [j_off, w_off]]:
            vert_mat[i] += weights[wi] * skin_mats[jj]
        pos += c

    # ---- geometry
    geom_id = skin.get('source').lstrip('#')
    geom = root.find(f".//{C}geometry[@id='{geom_id}']")
    mesh = geom.find(f'{C}mesh')
    pos_src_id = mesh.find(f'{C}vertices/{C}input[@semantic="POSITION"]').get('source')
    pos_arr = source_by_id(mesh, pos_src_id).find(f'{C}float_array')
    P = floats(pos_arr).reshape(-1, 3)
    assert len(P) == nverts, (len(P), nverts)
    Ph = np.c_[P, np.ones(len(P))]
    Pn = np.einsum('nij,nj->ni', vert_mat, Ph)[:, :3]
    pos_arr.text = ' '.join(f'{x:.6g}' for x in Pn.ravel())

    # normals are indexed per polygon corner; map each normal to a vertex to get its matrix
    nrm_arr = None
    normal_vertex = {}
    for prim in mesh:
        if prim.tag.replace(C, '') not in ('polylist', 'triangles', 'polygons'):
            continue
        inputs = prim.findall(f'{C}input')
        pstride = max(int(i.get('offset')) for i in inputs) + 1
        voff = int(next(i for i in inputs if i.get('semantic') == 'VERTEX').get('offset'))
        ninp = next((i for i in inputs if i.get('semantic') == 'NORMAL'), None)
        if ninp is None:
            continue
        nrm_arr = source_by_id(mesh, ninp.get('source')).find(f'{C}float_array')
        noff = int(ninp.get('offset'))
        idx = np.array(prim.find(f'{C}p').text.split(), dtype=int).reshape(-1, pstride)
        for vi, ni in idx[:, [voff, noff]]:
            normal_vertex.setdefault(int(ni), int(vi))
    if nrm_arr is not None:
        N = floats(nrm_arr).reshape(-1, 3)
        for ni, vi in normal_vertex.items():
            n = vert_mat[vi, :3, :3] @ N[ni]
            N[ni] = n / (np.linalg.norm(n) or 1.0)
        nrm_arr.text = ' '.join(f'{x:.6g}' for x in N.ravel())

    # ---- strip skinning/animation, instance the geometry directly at identity
    for lib in ('library_controllers', 'library_animations'):
        el = root.find(f'{C}{lib}')
        if el is not None:
            root.remove(el)
    for parent in list(scene.iter(f'{C}node')):
        for k in list(parent.findall(f'{C}node')):
            if k.get('type') == 'JOINT':
                parent.remove(k)
    for n in list(scene.findall(f'{C}node')):
        ic = n.find(f'{C}instance_controller')
        if ic is None:
            if not n.findall(f'{C}node') and n.find(f'{C}instance_geometry') is None:
                scene.remove(n)  # empty armature node
            continue
        ig = ET.Element(f'{C}instance_geometry', {'url': '#' + geom_id, 'name': n.get('name', '')})
        bm = ic.find(f'{C}bind_material')
        if bm is not None:
            ig.append(copy.deepcopy(bm))
        for t in list(n):
            n.remove(t)  # baked positions are already in scene frame
        n.append(ig)

    tree.write(args.dst, xml_declaration=True, encoding='utf-8')
    lo, hi = Pn.min(0), Pn.max(0)
    print(f'baked frame {args.frame}: bbox min {lo.round(3)} max {hi.round(3)}')


if __name__ == '__main__':
    main()
