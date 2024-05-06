#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

int main()
{
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;
  // Add pose stream
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  // Start pipeline with chosen configuration
  pipe.start(cfg);


  while (true)
  {
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
    
    std::cout << "\r" << "Device Position: " <<pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z<< " (meters)";
    
  }
    return 0;
}
