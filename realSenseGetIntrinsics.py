#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import math
import pyrealsense2 as rs


pipeline = rs.pipeline()
config = rs.config()

#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

profile = pipeline.start(config)

rgb_stream = profile.get_stream(rs.stream.color)
rgb_stream_profile = rs.video_stream_profile(rgb_stream)
rgb_intrinsics = rgb_stream_profile.get_intrinsics()

print()

print("Color (RGB) Camera")
print("------------------")
print("Resolution:  %d x %d" % (rgb_intrinsics.width, rgb_intrinsics.height))
print("Principal Point:  %.03f, %.03f" % (rgb_intrinsics.ppx, rgb_intrinsics.ppy))
print("Focal Length: %.03f, %.03f" % (rgb_intrinsics.fx, rgb_intrinsics.fy))
#rgb_hfov = 2 * math.atan(rgb_intrinsics.width / 2.0 / rgb_intrinsics.fx)
#rgb_vfov = 2 * math.atan(rgb_intrinsics.height / 2.0 / rgb_intrinsics.fy)
#print("HFoV: %.03f" % (math.degrees(rgb_hfov)))
#print("VFoV: %.03f" % (math.degrees(rgb_vfov)))
rgb_hfov, rgb_vfov = rs.rs2_fov(rgb_intrinsics)
print("HFoV: %.03f, VFoV %.03f" % (rgb_hfov, rgb_vfov))
print()


d_stream = profile.get_stream(rs.stream.depth)
d_stream_profile = rs.video_stream_profile(d_stream)
d_intrinsics = d_stream_profile.get_intrinsics()

print("Depth Camera")
print("------------")
print("Resolution:  %d x %d" % (d_intrinsics.width, d_intrinsics.height))
print("Principal Point:  %.03f, %.03f" % (d_intrinsics.ppx, d_intrinsics.ppy))
print("Focal Length: %.03f, %.03f" % (d_intrinsics.fx, d_intrinsics.fy))
#d_hfov = 2 * math.atan(d_intrinsics.width / 2.0 / d_intrinsics.fx)
#d_vfov = 2 * math.atan(d_intrinsics.height / (2.0 * d_intrinsics.fy))
#print("HFoV: %.03f" % (math.degrees(d_hfov)))
#print("VFoV: %.03f" % (math.degrees(d_vfov)))
d_hfov, d_vfov = rs.rs2_fov(d_intrinsics)
print("HFoV: %.03f, VFoV %.03f" % (d_hfov, d_vfov))

print()


pipeline.stop()



