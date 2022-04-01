#!/usr/bin/env python3

from cloversim.generation import World, Include
from cloversim.generation import ArucoMap, generate_aruco_map
from cloversim.generation import Box
from cloversim.generation import ImageTextures
import qrcode
import cv2

WORLD = World()
WORLD.add(Include("model://sun"))
WORLD.add(Include("model://parquet_plane", pose=(0, 0, -0.01)))

qrcode_texture = qrcode.make("1.5 0.3\n2.5 2.5").get_image()
image_textures = ImageTextures({
    "qrcode":
    qrcode_texture,
    "field":
    cv2.imread("/home/clover/catkin_ws/src/fr_task/field.png")
})
image_textures.generate_materials()

WORLD.add(
    Box("field",
        size=(4.22, 3.22, 0.001),
        pose=(2.11, -1.61, 0.001),
        mass=0.1,
        material=image_textures["field"],
        static=True))

WORLD.add(
    Box("qrcode",
        size=(0.35, 0.35, 0.001),
        pose=(-0.35, 0, 0.01),
        mass=0.1,
        material=image_textures["qrcode"],
        static=True))

aruco_map = """
# id    length  x       y       z       rot_z   rot_y   rot_x
115     0.22    0.0     0.0     0       0       0       0
119     0.22    1.0     0.0     0       0       0       0
120     0.22    2.0     0.0     0       0       0       0
134     0.22    3.0     0.0     0       0       0       0
110     0.22    0.0     1.0     0       0       0       0
130     0.22    1.0     1.0     0       0       0       0
112     0.22    2.0     1.0     0       0       0       0
108     0.22    3.0     1.0     0       0       0       0
121     0.22    0.0     2.0     0       0       0       0
129     0.22    1.0     2.0     0       0       0       0
122     0.22    2.0     2.0     0       0       0       0
109     0.22    3.0     2.0     0       0       0       0
131     0.22    0.0     3.0     0       0       0       0
124     0.22    1.0     3.0     0       0       0       0
132     0.22    2.0     3.0     0       0       0       0
133     0.22    3.0     3.0     0       0       0       0
116     0.22    0.0     4.0     0       0       0       0
123     0.22    1.0     4.0     0       0       0       0
111     0.22    2.0     4.0     0       0       0       0
140     0.22    3.0     4.0     0       0       0       0
"""
w = open("/home/clover/catkin_ws/src/clover/aruco_pose/map/map.txt", "w")
w.write(aruco_map)
w.close()

# Write your world generation code hereFix bug fix of seeding
