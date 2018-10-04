#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")


import time
import pyrealsense2 as rs


pipeline = rs.pipeline()
config = rs.config()

#config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

config.enable_record_to_file(sys.argv[1])

profile = pipeline.start(config)

time.sleep(20)

