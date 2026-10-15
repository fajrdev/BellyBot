#!/usr/bin/env python3
"""Fix white material in Gazebo Harmonic simulation for all environment and conveyor models.

Root Cause:
The original CAD DAE exporter generated COLLADA 1.4.1 files without vertex normals
and without ambient lighting channels. Gazebo Harmonic's Ogre 2 rendering engine
relies on TangentSpaceCalc for PBS (Physically Based Shading). When vertex normals
are missing, TangentSpaceCalc throws an exception, aborts material creation,
and falls back to solid white.

This script:
1. Computes vertex normals from face geometry for every submesh in every DAE file.
2. Injects the <source id="...normals"> and <input semantic="NORMAL"> into <vertices>.
3. Injects proper <ambient> and <specular> channels into <lambert> / <phong> effects.
"""

import os
import glob
import xml.etree.ElementTree as ET
import numpy as np

NS_URI = 'http://www.collada.org/2005/11/COLLADASchema'
NS = {'c': NS_URI}
ET.register_namespace('', NS_URI)

def fix_dae_file(dae_path):
    tree = ET.parse(dae_path)
    root = tree.getroot()
    modified = False

    # 1. Fix lighting effects: Add ambient and specular if missing
    for effect in root.findall('.//c:effect', NS):
        technique = effect.find('.//c:technique', NS)
        if technique is None:
            continue
        
        shader = technique.find('c:lambert', NS)
        if shader is None:
            shader = technique.find('c:phong', NS)
        if shader is None:
            continue

        diffuse = shader.find('c:diffuse', NS)
        ambient = shader.find('c:ambient', NS)
        specular = shader.find('c:specular', NS)

        if diffuse is not None:
            color = diffuse.find('c:color', NS)
            if color is not None:
                c_vals = [float(x) for x in color.text.split()]
                if len(c_vals) >= 3:
                    # Add ambient if missing
                    if ambient is None:
                        amb_elem = ET.Element(f'{{{NS_URI}}}ambient')
                        amb_col = ET.SubElement(amb_elem, f'{{{NS_URI}}}color')
                        # 45% diffuse brightness provides rich shadow depth without washed-out glare
                        amb_col.text = f'{c_vals[0]*0.45:.6f} {c_vals[1]*0.45:.6f} {c_vals[2]*0.45:.6f} 1.000000'
                        shader.insert(0, amb_elem)
                        modified = True

                    # Add specular if missing
                    if specular is None:
                        spec_elem = ET.Element(f'{{{NS_URI}}}specular')
                        spec_col = ET.SubElement(spec_elem, f'{{{NS_URI}}}color')
                        spec_col.text = '0.150000 0.150000 0.150000 1.000000'
                        shader.append(spec_elem)
                        modified = True

    # 2. Add vertex normals to each geometry
    for geom in root.findall('.//c:geometry', NS):
        mesh = geom.find('c:mesh', NS)
        if mesh is None:
            continue

        pos_source = mesh.find('c:source', NS)
        if pos_source is None:
            continue

        pos_id = pos_source.attrib.get('id', '')
        norm_id = pos_id.replace('positions', 'normals')
        if not norm_id.endswith('normals'):
            norm_id = norm_id + '_normals'

        # Check if normals already exist
        existing_norm = mesh.find(f".//c:source[@id='{norm_id}']", NS)
        vertices = mesh.find('c:vertices', NS)
        has_norm_input = vertices is not None and vertices.find(f".//c:input[@semantic='NORMAL']", NS) is not None

        if existing_norm is not None and has_norm_input:
            continue

        fa = pos_source.find('.//c:float_array', NS)
        if fa is None or not fa.text:
            continue

        coords = np.array([float(x) for x in fa.text.split()], dtype=np.float64).reshape(-1, 3)
        n_verts = len(coords)

        triangles = mesh.find('c:triangles', NS)
        if triangles is None:
            continue

        p_elem = triangles.find('c:p', NS)
        if p_elem is None or not p_elem.text:
            continue

        indices = np.array([int(x) for x in p_elem.text.split()], dtype=np.int32).reshape(-1, 3)

        # Compute area-weighted smooth vertex normals
        vert_normals = np.zeros_like(coords)
        for tri in indices:
            v0, v1, v2 = coords[tri[0]], coords[tri[1]], coords[tri[2]]
            fn = np.cross(v1 - v0, v2 - v0)
            vert_normals[tri[0]] += fn
            vert_normals[tri[1]] += fn
            vert_normals[tri[2]] += fn

        norm_lens = np.linalg.norm(vert_normals, axis=1, keepdims=True)
        norm_lens[norm_lens < 1e-9] = 1.0
        vert_normals = vert_normals / norm_lens

        # Build normal source
        if existing_norm is None:
            norm_source = ET.Element(f'{{{NS_URI}}}source', {'id': norm_id})
            norm_fa = ET.SubElement(norm_source, f'{{{NS_URI}}}float_array', {
                'id': f'{norm_id}-array',
                'count': str(n_verts * 3)
            })
            norm_fa.text = ' '.join(f'{x:.6f}' for x in vert_normals.flatten())

            tech = ET.SubElement(norm_source, f'{{{NS_URI}}}technique_common')
            accessor = ET.SubElement(tech, f'{{{NS_URI}}}accessor', {
                'source': f'#{norm_id}-array',
                'count': str(n_verts),
                'stride': '3'
            })
            for ax in ['X', 'Y', 'Z']:
                ET.SubElement(accessor, f'{{{NS_URI}}}param', {'name': ax, 'type': 'float'})

            idx = list(mesh).index(pos_source)
            mesh.insert(idx + 1, norm_source)

        if not has_norm_input and vertices is not None:
            ET.SubElement(vertices, f'{{{NS_URI}}}input', {
                'semantic': 'NORMAL',
                'source': f'#{norm_id}'
            })

        modified = True

    if modified:
        tree.write(dae_path, encoding='utf-8', xml_declaration=True)
        return True
    return False

def main():
    base_dir = '/home/fajrda/sutd-hackathon/BellyBot_Aircraft_Environment_v2/ros2/bellybot_aircraft_environment/models'
    dae_files = sorted(glob.glob(os.path.join(base_dir, '**', '*.dae'), recursive=True))

    print(f"Found {len(dae_files)} COLLADA files to process in {base_dir}...")
    fixed_count = 0
    for dae in dae_files:
        rel = os.path.relpath(dae, base_dir)
        try:
            if fix_dae_file(dae):
                print(f"  [FIXED] {rel}")
                fixed_count += 1
            else:
                print(f"  [OK]    {rel}")
        except Exception as e:
            print(f"  [ERR]   {rel}: {e}")

    print(f"\nCompleted: {fixed_count}/{len(dae_files)} files upgraded with vertex normals and ambient lighting.")

if __name__ == '__main__':
    main()
