// mm; affine-hexagonal outer proxy, NOT measured tip geometry
L=25.0; W=5.0; H=4.0;
// Local CAD extrusion is Z; rotate so the model long axis is X.
rotate([0,90,0]) linear_extrude(height=L,center=true)
polygon([[0,W/2],[-H/2,W/4],[-H/2,-W/4],[0,-W/2],[H/2,-W/4],[H/2,W/4]]);
