#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import pyrealsense2 as rs


pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

#config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

profile = pipeline.start(config)

rgb_stream = profile.get_stream(rs.stream.color)
rgb_stream_profile = rs.video_stream_profile(rgb_stream)
rgb_intrinsics = rgb_stream_profile.get_intrinsics()

print("Color (RGB) Camera")
print("------------------")
print("Resolution:  %d x %d" % (rgb_intrinsics.width, rgb_intrinsics.height))
print("Principal Point:  %.03f, %.03f" % (rgb_intrinsics.ppx, rgb_intrinsics.ppy))
print("Focal Length: %.03f, %.03f" % (rgb_intrinsics.fx, rgb_intrinsics.fy))
print()


d_stream = profile.get_stream(rs.stream.depth)
d_stream_profile = rs.video_stream_profile(d_stream)
d_intrinsics = d_stream_profile.get_intrinsics()

print("Depth Camera")
print("------------")
print("Resolution:  %d x %d" % (d_intrinsics.width, d_intrinsics.height))
print("Principal Point:  %.03f, %.03f" % (d_intrinsics.ppx, d_intrinsics.ppy))
print("Focal Length: %.03f, %.03f" % (d_intrinsics.fx, d_intrinsics.fy))
print()

