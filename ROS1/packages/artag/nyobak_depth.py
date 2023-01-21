## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs
import cv2
import numpy as np

# out = np.empty((480, 640, 3), dtype=np.uint8)
try:
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()

    # Configure streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)
    
    while True:
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()

        vis = np.zeros((480, 640), np.uint8)
        vis2 = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        for y in range(480):
            for x in range(640):
                dist = depth.get_distance(x, y)
                vis2[y][x] = dist 
                # print(dist) 255 // 3 3 / 255

        vis3 = (vis2[213:427][160-320].astype('float32')) 
        print(f'dist: {np.mean(vis3)}')

        cv2.imshow("depth", vis2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    exit(0)
except Exception as e:
    print(e)
    pass
