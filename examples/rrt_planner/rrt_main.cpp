#include "rrt_main.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <lcm/lcm-cpp.hpp>
#include "lidar_raw.hpp"
#include "obst_map.hpp"

#define LIDAR_BUFFER_SIZE 1500

#define GRID_X_SIZE 200
#define GRID_X_M 10 // in m
#define GRID_Y_SIZE 200
#define GRID_Y_M 10 // in m
#define LIDAR_DIST_MAX 9500 // in mm
#define OBST_FILL 40 // in mm
#define CONVERT_STEP 15

namespace pln = planner;

class obst_map_handler 
{
    public:

    // lidar data array
    bool obst_map[GRID_X_SIZE][GRID_Y_SIZE];


        obst_map_handler() {}

        ~obst_map_handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const exlcm::obst_map* msg);

    private:

};

void obst_map_handler::handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const exlcm::obst_map* msg)
{
    printf("Received message on channel \"%s\":\n", chan.c_str());
    printf("  timestamp   = %lld\n", (long long)msg->timestamp);
    printf("  enabled     = %d\n", msg->enabled);

    // copy in obst mask
    for (int i = 0; i < GRID_X_SIZE; i++) 
        for (int j = 0; j < GRID_X_SIZE; j++)
                obst_map[i][j] = msg->obst_map[i][j];
}



int main() {
  const int DIM = 2;
  const std::string PARAM_YAML_DIRECTORY = "../examples/rrt_planner/data";
  const std::string PARAM_YAML_FILE_NAME = "param.yaml";
  const std::string PARAM_YAML_FILE_PATH = PARAM_YAML_DIRECTORY + "/" + PARAM_YAML_FILE_NAME;

  try {
    std::unique_ptr<pln::base::PlannerBase> planner;
    std::shared_ptr<pln::base::ConstraintBase> constraint;

    RRTParam rrt_param;
    RRTStarParam rrt_star_param;
    InformedRRTStarParam informed_rrt_star_param;
    cvlib::YAMLHelper::readStruct(rrt_param, PARAM_YAML_FILE_PATH, "RRT");
    cvlib::YAMLHelper::readStruct(rrt_star_param, PARAM_YAML_FILE_PATH, "RRTStar");
    cvlib::YAMLHelper::readStruct(informed_rrt_star_param, PARAM_YAML_FILE_PATH, "InformedRRTStar");

    StateParam world_size, start_state, goal_state;
    cvlib::YAMLHelper::readStruct(world_size, PARAM_YAML_FILE_PATH, "WorldSize");
    cvlib::YAMLHelper::readStruct(start_state, PARAM_YAML_FILE_PATH, "StartState");
    cvlib::YAMLHelper::readStruct(goal_state, PARAM_YAML_FILE_PATH, "GoalState");

    auto img_terminate_search_cost = cvlib::YAMLHelper::read<double>(PARAM_YAML_FILE_PATH, "ImageTerminateSearchCost");
    auto obs_terminate_search_cost =
        cvlib::YAMLHelper::read<double>(PARAM_YAML_FILE_PATH, "ObstaclesTerminateSearchCost");
    auto terminate_search_cost = img_terminate_search_cost;

    while (true) {
      bool end_flag = false;
      cv::Mat world = cv::Mat::zeros(cv::Size(world_size.x, world_size.y), CV_8UC1);
      auto start = pln::State(start_state.x, start_state.y);
      auto goal = pln::State(goal_state.x, goal_state.y);

      [&]() {
        while (true) {
          char mode = '0';
          std::cout << "[1] Please choose constraint type" << std::endl;
          std::cout << "1. image" << std::endl;
          std::cout << "2. set of circle" << std::endl;
          std::cout << "q. quit" << std::endl << std::endl;
          std::cout << "mode : ";
          std::cin >> mode;
          std::cout << std::endl;

          switch (mode) {
            case '1': {
              //--- set constraint based on image
              auto img_file_path = PARAM_YAML_DIRECTORY + "/" +
                                   cvlib::YAMLHelper::read<std::string>(PARAM_YAML_FILE_PATH, "ConstraintImage");

              world = cv::imread(img_file_path, CV_8UC1);

              //-- generate space and constraint
              // A space should set bound at each dimension
              pln::EuclideanSpace space(DIM);
              std::vector<pln::Bound> bounds{pln::Bound(0, world.cols), pln::Bound(0, world.rows)};
              space.setBound(bounds);

              // A constraint based on image use one dimension
              // std::vector<pln::ConstraintType>, and you should set each
              // dimension size to std::vector<uint32_t>
              std::vector<pln::ConstraintType> map(world.cols * world.rows, pln::ConstraintType::ENTAERABLE);
              for (int yi = 0; yi < world.rows; yi++) {
                for (int xi = 0; xi < world.cols; xi++) {
                  if (world.data[xi + yi * world.cols] != 0) {
                    map[xi + yi * world.cols] = pln::ConstraintType::NOENTRY;
                  }
                }
              }

              std::vector<uint32_t> each_dim_size{(uint32_t)world.cols, (uint32_t)world.rows};

              //-- apply constraint
              constraint = std::make_shared<pln::GridConstraint>(space, map, each_dim_size);

              //--- set terminate cost
              terminate_search_cost = img_terminate_search_cost;
              return;
            }
            case '2': {
              //--- set constraint based on set of circle
              //-- generate space and constraint
              // A space should set bound at each dimension
              pln::EuclideanSpace space(DIM);
              std::vector<pln::Bound> bounds{pln::Bound(0, world.cols), pln::Bound(0, world.rows)};
              space.setBound(bounds);

              // set of circle define as
              // std::vector<pln::PointCloudConstraint::Hypersphere>
              auto obstacle_num = cvlib::YAMLHelper::read<int>(PARAM_YAML_FILE_PATH, "Obstacles", "num");
              std::vector<pln::PointCloudConstraint::Hypersphere> obstacles;
              for (int i = 0; i < obstacle_num; i++) {
                ObstacleParam obstacle_param;
                cvlib::YAMLHelper::readStruct(obstacle_param, PARAM_YAML_FILE_PATH, "Obstacles",
                                              "Obstacle" + std::to_string(i));
                obstacles.emplace_back(pln::State(obstacle_param.x, obstacle_param.y), obstacle_param.radius);
              }

              //-- apply constraint
              constraint = std::make_shared<pln::PointCloudConstraint>(space, obstacles);

              //-- draw set of circle to world img
              for (const auto &obstacle : static_cast<pln::PointCloudConstraint *>(constraint.get())->getRef()) {
                cv::circle(world, cv::Point(obstacle.getState().vals[0], obstacle.getState().vals[1]),
                           obstacle.getRadius(), 0xFF, -1, cv::LINE_AA);
              }

              //--- set terminate cost
              terminate_search_cost = obs_terminate_search_cost;
              return;
            }
            case 'q': {
              end_flag = true;
              return;
            }
            default: {
              break;
            }
          }
        }
      }();

      if (end_flag) {
        break;
      }

      [&]() {
        while (true) {
          char mode = '0';
          std::cout << "[2] Please choose method of path planning" << std::endl;
          std::cout << "1. RRT" << std::endl;
          std::cout << "2. RRT*" << std::endl;
          std::cout << "3. Informed-RRT*" << std::endl;
          std::cout << "q. quit" << std::endl << std::endl;
          std::cout << "mode : ";
          std::cin >> mode;
          std::cout << std::endl;

          switch (mode) {
            case '1': {
              planner = std::make_unique<pln::RRT>(DIM, rrt_param.max_sampling_num, rrt_param.goal_sampling_rate,
                                                   rrt_param.expand_dist);
              return;
            }
            case '2': {
              planner = std::make_unique<pln::RRTStar>(DIM, rrt_star_param.max_sampling_num,
                                                       rrt_star_param.goal_sampling_rate, rrt_star_param.expand_dist,
                                                       rrt_star_param.R);
              return;
            }
            case '3': {
              planner = std::make_unique<pln::InformedRRTStar>(
                  DIM, informed_rrt_star_param.max_sampling_num, informed_rrt_star_param.goal_sampling_rate,
                  informed_rrt_star_param.expand_dist, informed_rrt_star_param.R,
                  informed_rrt_star_param.goal_region_radius);
              return;
            }
            case 'q': {
              end_flag = true;
              return;
            }
            default: {
              break;
            }
          }
        }
      }();

      if (end_flag) {
        break;
      }


    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    std::cout<<"lcm init done\n"<<std::endl;

    obst_map_handler *handlerObject = new obst_map_handler();
    lcm.subscribe("OBST_MAP", &obst_map_handler::handleMessage, handlerObject);

    while(0 == lcm.handle())
    {

        // visualize
        cv::Mat world = cv::Mat::zeros(cv::Size(world_size.x, world_size.y), CV_8UC1);
        cv::cvtColor(world, world, cv::COLOR_BGR2RGB);


        std::cout<<"rrt loop"<<std::endl;

        // definition of obstacle (point cloud type)
        std::vector<pln::PointCloudConstraint::Hypersphere> obstacles_r;

        for (int i = GRID_X_SIZE/2-200; i < GRID_X_SIZE/2+200; i++) 
        {
            for (int j = GRID_Y_SIZE/2-GRID_Y_SIZE/4; j < GRID_Y_SIZE/2+GRID_Y_SIZE/4; j++)
            {
              if(handlerObject->obst_map[i][j]==1)
              {
                double p_obst_x = i/double(GRID_X_SIZE)*world_size.x;
                double p_obst_y = j/double(GRID_Y_SIZE)*world_size.y;
                obstacles_r.emplace_back(pln::State(p_obst_x, p_obst_y),  1.0); // x : 10.0, y : 20.0, radius : 10.0
              }
            }
        }


      pln::EuclideanSpace space(DIM);
      std::vector<pln::Bound> bounds{pln::Bound(0, world.cols), pln::Bound(0, world.rows)};
      space.setBound(bounds);

      // definition of constraint using std::shared_ptr
      auto constraint_updated = std::make_shared<pln::PointCloudConstraint>(space, obstacles_r);
      std::cout<<"obst init done\n"<<std::endl;

      //-- draw set of circle to world img
      for (const auto &obstacles_r : static_cast<pln::PointCloudConstraint *>(constraint_updated.get())->getRef()) {
        cv::circle(world, cv::Point(obstacles_r.getState().vals[0], obstacles_r.getState().vals[1]),
                    obstacles_r.getRadius(), 0xFF, -1, cv::LINE_AA);
      }

      planner->setProblemDefinition(constraint_updated);
      planner->setTerminateSearchCost(terminate_search_cost);

      auto start_time = std::chrono::system_clock::now();
      bool status = planner->solve(start, goal);
      auto end_time = std::chrono::system_clock::now();

      if (status) 
      {
        // draw and output result
        auto node_list = planner->getNodeList();
        auto result = planner->getResult();

        for (int yi = 0; yi < world.rows; yi++) {
          for (int xi = 0; xi < world.cols; xi++) {
            if (world.at<cv::Vec3b>(yi, xi) != cv::Vec3b(0, 0, 0)) {
              world.at<cv::Vec3b>(yi, xi) = cv::Vec3b(192, 128, 224);
            }
          }
        }

        auto leafs = node_list->searchLeafs();
        for (auto node : leafs) {
          while (node->parent != nullptr) {
            cv::line(world, cv::Point(node->state.vals[0], node->state.vals[1]),
                     cv::Point(node->parent->state.vals[0], node->parent->state.vals[1]), cv::Vec3b(64, 92, 16), 1.0,
                     cv::LINE_AA);
            node = node->parent;
          }
        }

        auto prev_node_pos = cv::Point(result[0].vals[0], result[0].vals[1]);
        for (const auto &r : result) {
          std::cout << r << std::endl;

          cv::circle(world, cv::Point(r.vals[0], r.vals[1]), 3.0, cv::Vec3b(128, 128, 255), 1, cv::LINE_AA);
          cv::line(world, cv::Point(r.vals[0], r.vals[1]), prev_node_pos, cv::Vec3b(128, 255, 128), 1, cv::LINE_AA);
          prev_node_pos = cv::Point(r.vals[0], r.vals[1]);
        }

        cv::circle(world, cv::Point(result.front().vals[0], result.front().vals[1]), 6.0, cv::Vec3b(128, 255, 255), -1,
                   cv::LINE_AA);
        cv::putText(world, "Start", cv::Point(result.front().vals[0] + 10, result.front().vals[1]),
                    cv::FONT_HERSHEY_COMPLEX_SMALL | cv::FONT_ITALIC, 1.0, cv::Scalar(128, 255, 255), 1, cv::LINE_AA);

        cv::circle(world, cv::Point(result.back().vals[0], result.back().vals[1]), 6.0, cv::Vec3b(128, 255, 255), -1,
                   cv::LINE_AA);
        cv::putText(world, "Goal", cv::Point(result.back().vals[0] + 10, result.back().vals[1]),
                    cv::FONT_HERSHEY_COMPLEX_SMALL | cv::FONT_ITALIC, 1.0, cv::Scalar(128, 255, 255), 1, cv::LINE_AA);

        cv::namedWindow("world", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::imshow("world", world);
        cv::waitKey(1);

        //cv::destroyWindow("world");

        std::cout << "total cost : " << planner->getResultCost() << std::endl;
      } 
      else 
      {
        std::cout << "Could not find path" << std::endl;
      }



      std::cout << "elapsed time : "
                << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << "ms"
                << std::endl
                << std::endl;
    }
    }
  
  } 
  catch (const std::exception &e) 
  {
    std::cout << e.what() << std::endl;
    exit(1);
  }
}


