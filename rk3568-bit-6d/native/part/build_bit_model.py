#!/usr/bin/env python3
"""Export a measured shaft plus a clearly marked approximate tapered head envelope."""
import argparse
import json
import math
from pathlib import Path


def mesh(length, width, height, head_length=None, tip_ratio=.25):
    if not all(math.isfinite(v) and v > 0 for v in (length, width, height)):
        raise ValueError('Dimensions must be finite and positive')
    # An affine hexagon preserves both reported extents. A regular hexagon with
    # 4 mm across flats would be 4.619 mm across corners, not exactly 5 mm.
    section = [(width/2, 0), (width/4, height/2), (-width/4, height/2),
               (-width/2, 0), (-width/4, -height/2), (width/4, -height/2)]
    rings = [(-length/2, 1), (length/2, 1)]
    if head_length is not None:
        if not math.isfinite(head_length) or not 0 < head_length < length or not 0 < tip_ratio <= 1:
            raise ValueError('Invalid head length or taper ratio')
        rings = [(-length/2, 1), (length/2-head_length, 1), (length/2, tip_ratio)]
    vertices = [(x, y*scale, z*scale) for x, scale in rings for y, z in section]
    faces = []
    last = len(vertices)-6
    for j in range(1, 5):
        faces.extend([(0, j+1, j), (last, last+j, last+j+1)])
    for ring in range(len(rings)-1):
        offset = ring*6
        for i in range(6):
            j = (i+1) % 6
            faces.extend([(offset+i, offset+j, offset+j+6), (offset+i, offset+j+6, offset+i+6)])
    return vertices, faces


def normal(a, b, c):
    u = [b[i]-a[i] for i in range(3)]
    v = [c[i]-a[i] for i in range(3)]
    n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
    length = math.sqrt(sum(q*q for q in n))
    return [q/length for q in n]


def export(dimensions, output):
    length = dimensions['total_length_mm']
    width = dimensions['endface_corner_to_corner_width_mm']
    height = dimensions['endface_parallel_flat_height_mm']
    head = dimensions.get('head_length_mm')
    ratio = dimensions.get('tip_width_ratio_assumed', .25)
    if head is not None and abs(dimensions.get('shaft_length_mm', length-head)+head-length) > 1e-6:
        raise ValueError('Shaft plus head must equal total length')
    vertices, faces = mesh(length, width, height, head, ratio)
    output.mkdir(parents=True, exist_ok=False)
    obj = ['# Units mm; outer proxy only; NO measured tip geometry', 'o bit_outer_proxy']
    obj += ['v ' + ' '.join(map(str, p)) for p in vertices]
    obj += ['f ' + ' '.join(str(i+1) for i in face) for face in faces]
    (output / 'bit_outer_proxy.obj').write_text('\n'.join(obj)+'\n')
    stl = ['solid bit_outer_proxy_mm']
    for face in faces:
        a, b, c = [vertices[i] for i in face]
        stl += ['facet normal ' + ' '.join(map(str, normal(a, b, c))), ' outer loop']
        stl += ['  vertex ' + ' '.join(map(str, p)) for p in (a, b, c)]
        stl += [' endloop', 'endfacet']
    stl += ['endsolid bit_outer_proxy_mm']
    (output / 'bit_outer_proxy.stl').write_text('\n'.join(stl)+'\n')
    scad = ('// mm; head is a tapered envelope, NOT a measured Phillips surface.\n'
            '// +X points to tip. Endpoint scale is an explicit unmeasured assumption.\n'
            'polyhedron(points=' + json.dumps(vertices) + ', faces=' + json.dumps(faces) + ', convexity=10);\n')
    (output / 'bit_outer_proxy.scad').write_text(scad)
    metadata = {**dimensions, 'geometry': 'affine_hexagonal_shaft_with_tapered_head_envelope' if head else 'affine_hexagonal_outer_proxy',
                'tip_geometry_modeled': False, 'pnp_landmarks_available': False,
                'usage': 'Visualization/envelope only; NOT a validated 6D registration model.',
                'regular_hexagon_inferred_corner_width_mm': height*2/math.sqrt(3)}
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dimensions', type=Path, default=Path(__file__).with_name('dimensions.json'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    export(json.loads(args.dimensions.read_text()), args.output)
    print(args.output)
