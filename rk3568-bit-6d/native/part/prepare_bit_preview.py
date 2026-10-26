#!/usr/bin/env python3
"""Reuse the measured proxy mesh as a millimetre-to-metre preview, not a detector."""
import argparse
import json
from pathlib import Path
from build_bit_model import mesh


def prepare(dimensions, assembly):
    vertices, _ = mesh(dimensions['total_length_mm'], dimensions['endface_corner_to_corner_width_mm'],
        dimensions['endface_parallel_flat_height_mm'], dimensions['head_length_mm'],
        dimensions['tip_width_ratio_assumed'])
    edges=[]
    rings=len(vertices)//6
    for ring in range(rings):
        for i in range(6):
            edges.extend([ring*6+i,ring*6+(i+1)%6])
            if ring+1<rings:edges.extend([ring*6+i,(ring+1)*6+i])
    assembly={**assembly,'preview_geometry':dict(vertices_m=[v/1000 for p in vertices for v in p],
        edges=edges,source='measured shaft with approximate head envelope; visual alignment only')}
    assert len(vertices)==18 and max(v[0] for v in vertices)-min(v[0] for v in vertices)==25
    assert len(edges)%2==0 and all(0<=i<len(vertices) for i in edges)
    return assembly


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    result=prepare(json.loads((root/'dimensions.json').read_text()),json.loads((root/'assembly_marker42.json').read_text()))
    with args.output.open('x') as f:json.dump(result,f,indent=2)
