"""Check dimensions, closed surface, positive volume and winding of the CAD proxy."""
from collections import Counter
from build_bit_model import mesh

vertices, faces = mesh(25, 5, 4)
assert [max(p[i] for p in vertices)-min(p[i] for p in vertices) for i in range(3)] == [25, 5, 4]
edges = Counter(tuple(sorted((f[i], f[(i+1) % 3]))) for f in faces for i in range(3))
assert set(edges.values()) == {2}
assert len(vertices) - len(edges) + len(faces) == 2
volume = 0
for face in faces:
    a, b, c = [vertices[i] for i in face]
    volume += (a[0]*(b[1]*c[2]-b[2]*c[1]) + a[1]*(b[2]*c[0]-b[0]*c[2])
               + a[2]*(b[0]*c[1]-b[1]*c[0])) / 6
assert abs(volume - 375) < 1e-8
vertices, faces = mesh(25, 5, 4, 8, .25)
assert len(vertices) == 18 and len(faces) == 32
assert set(Counter(tuple(sorted((f[i], f[(i+1) % 3]))) for f in faces for i in range(3)).values()) == {2}
assert sorted({p[0] for p in vertices}) == [-12.5, 4.5, 12.5]
print('Closed 25x5x4 mm proxy mesh PASS')
