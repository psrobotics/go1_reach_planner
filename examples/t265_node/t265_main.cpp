#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <chrono>
#include <ctime>

// lcm
#include <lcm/lcm-cpp.hpp>
#include "motion_t.hpp"

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

  lcm::LCM lcm;
  if(!lcm.good())
  {
    std::cout << "lcm init failed" << std::endl;
    std::terminate();
  }
  // init a lcm type for vicon info passthrough
  exlcm::motion_t motion_t_msg;



  while (true)
  {
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
    
    std::cout << "\r" << "Device Position: " <<pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z<< " (meters)"<< std::endl;
    
    // copy in position data
    for(int s=0;s<3;s++)
    motion_t_msg.position[0] = -1*pose_data.translation.x;
    motion_t_msg.position[1] = pose_data.translation.z;
    motion_t_msg.position[2] = pose_data.translation.y;

    std::cout<<motion_t_msg.position<<std::endl;

    // copy in orientation data
    motion_t_msg.orientation[0] = pose_data.rotation.w;
    motion_t_msg.orientation[1] = pose_data.rotation.x;
    motion_t_msg.orientation[2] = pose_data.rotation.y;
    motion_t_msg.orientation[3] = pose_data.rotation.z;

    motion_t_msg.enabled = 1;

    // Get current time point
    auto now = std::chrono::system_clock::now();
    // Convert to milliseconds since the epoch
    auto millis_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    // Output the current timestamp in milliseconds
    std::cout << "Current timestamp in milliseconds: " << millis_since_epoch << std::endl;

    motion_t_msg.timestamp = millis_since_epoch;

    lcm.publish("T265_POSE", &motion_t_msg);
    std::cout << "lcm published t265 pose"<<std::endl;

  }
    return 0;
}
