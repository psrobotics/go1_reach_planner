import pyrealsense2 as rs
import numpy as np
import cv2

file_name = '/home/ps/projects/ral_fpv_vid/exp_mpc_3'
record_time = 70

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_record_to_file(file_name + '.bag')

# Start streaming
pipeline.start(config)

e1 = cv2.getTickCount()

# Define video writer parameters
output_video_path = file_name + '.mp4'
fps = 30
fourcc = cv2.VideoWriter_fourcc(*'MP4V')
# Define video writer object
out = cv2.VideoWriter(output_video_path, fourcc, fps, (640 * 2, 480))

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))
        out.write(images)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        e2 = cv2.getTickCount()
        t = (e2 - e1) / cv2.getTickFrequency()

        if t > record_time: # change it to record what length of video you are interested in
            print("Done!")
            break

finally:

    # Stop streaming
    pipeline.stop()
    out.release()