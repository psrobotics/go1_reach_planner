#include <iostream>
#include <string>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

// lcm
#include <lcm/lcm-cpp.hpp>
#include "motion_t.hpp"

int main(int argc, char **argv)
{
  if (argc < 3) {
    std::cerr << "Use ./motioncapture_example <type> <hostname> [option1] [value1] ..." << std::endl;
    return 1;
  }

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = argv[2];
  for (int i = 3; i+1 < argc; i+=2) {
    cfg[argv[i]] = argv[i+1];
  }

  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(argv[1], cfg);

  std::cout << "supportsRigidBodyTracking: " << mocap->supportsRigidBodyTracking() << std::endl;
  std::cout << "supportsLatencyEstimate: " << mocap->supportsLatencyEstimate() << std::endl;
  std::cout << "supportsPointCloud: " << mocap->supportsPointCloud() << std::endl;
  std::cout << "supportsTimeStamp: " << mocap->supportsTimeStamp() << std::endl;

  std::cout << "finishing init the motion tracking system" << std::endl;

  // Init the lcm server 
  // https://lcm-proj.github.io/lcm/content/tutorial-cpp.html
  lcm::LCM lcm;
  if(!lcm.good())
  {
    std::cout << "lcm init failed" << std::endl;
    std::terminate();
  }
  // init a lcm type for vicon info passthrough
  exlcm::motion_t motion_t_msg;

  // define the rigid body name we want to track
  std::string rb_name = "QUAD_RB_1";
  motion_t_msg.rb_name = rb_name;


  for (size_t frameId = 0;; ++frameId)
  {
    // Get a frame
    mocap->waitForNextFrame();

    std::cout << "frame " << frameId << std::endl;
    motion_t_msg.timestamp = frameId;

    // timestep
    if (mocap->supportsTimeStamp()) 
    {
      std::cout << "  timestamp: " << mocap->timeStamp() << " us" << std::endl;

    }

    // latency
    if (mocap->supportsLatencyEstimate()) 
    {
      std::cout << "  latency: " << std::endl;
      for (const auto& latency : mocap->latency()) {
        std::cout << "    " << latency.name() << " " << latency.value() << " s" << std::endl;
        motion_t_msg.latency = latency.value();
      }
    }

    // rigidbody_tracking
    if (mocap->supportsRigidBodyTracking()) 
    {
      auto rigidBodies = mocap->rigidBodies();

      std::cout << "  rigid bodies:" << std::endl;

      for (auto const& item: rigidBodies) 
      {
        const auto& rigidBody = item.second;

        std::cout << "    \"" << rigidBody.name() << "\":" << std::endl;

        if(rigidBody.name() == rb_name)
        {
          const auto& position = rigidBody.position();
          const auto& rotation = rigidBody.rotation();
          std::cout << "       position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
          std::cout << "       rotation: [" << rotation.w() << ", " << rotation.vec()(0) << ", "
                                              << rotation.vec()(1) << ", " << rotation.vec()(2) << "]" << std::endl;

          // copy in position data
          for(int s=0;s<3;s++)
            motion_t_msg.position[s] = position(s);

          // copy in orientation data
          motion_t_msg.orientation[0] = rotation.w();
          motion_t_msg.orientation[1] = rotation.vec()(0);
          motion_t_msg.orientation[2] = rotation.vec()(1);
          motion_t_msg.orientation[3] = rotation.vec()(2);

          motion_t_msg.enabled = 1;
        }
        else
        {
          motion_t_msg.enabled = 0;
        }
      }
    }

    // raw point cloud
    if (mocap->supportsPointCloud()) 
    {
      std::cout << "  pointcloud:" << std::endl;
      auto pointcloud = mocap->pointCloud();
      for (size_t i = 0; i < pointcloud.rows(); ++i) {
        const auto& point = pointcloud.row(i);
        std::cout << "    \"" << i << "\": [" << point(0) << "," << point(1) << "," << point(2) << "]" << std::endl;
      }
    }

    // lcm send info
    lcm.publish("VICON_LCM", &motion_t_msg);
    std::cout << "lcm data package sent" << std::endl;

  }

  return 0;
}
