#!/usr/bin/env python

from subprocess import call

call(["/usr/bin/meshlabserver", "-i", "points_raw.xyz", "-o", "points_mesh.ply", "-s", "xyz_to_mesh.mlx"])

call(["/usr/bin/meshlabserver", "-i", "points_mesh.ply", "-o", "points_mesh.xyz"])