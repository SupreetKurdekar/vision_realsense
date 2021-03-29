import pyrealsense2 as rs
import numpy as np
# Pointcloud persistency in case of dropped frames
pc = rs.pointcloud()
points = rs.points()

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
config = rs.config()

# This is the minimal recommended resolution for D435
config.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 90)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # Get aligned frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Tell pointcloud object to map to this color frame
        points = pc.calculate(depth_frame)
        pc.map_to(color_frame)
        vertices = np.array(points.get_vertices())
        tex_cood = np.array(points.get_texture_coordinates())
        print(tex_cood[0])
        # Generate the pointcloud and texture mappings
        # points = pc.calculate(depth_frame)

        # print(points)
        # ply = rs.save_to_ply("4.ply",pc)
        # ply.process()
        break

finally:
    pipeline.stop()