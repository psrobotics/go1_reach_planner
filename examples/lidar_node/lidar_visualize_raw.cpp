// vis lidar raw

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <lcm/lcm-cpp.hpp>
#include "lidar_raw.hpp"
#include "obst_map.hpp"
#include <opencv2/opencv.hpp>

#define LIDAR_BUFFER_SIZE 1500

#define GRID_X_SIZE 200
#define GRID_X_M 10 // in m
#define GRID_Y_SIZE 200
#define GRID_Y_M 10 // in m
#define LIDAR_DIST_MAX 9500 // in mm
#define OBST_FILL 40 // in mm
#define CONVERT_STEP 15
#define OBST_WALL_THICKNESS 200



class lidar_raw_handler 
{
    public:

    // lidar data array
    int count;
    double angle[LIDAR_BUFFER_SIZE];
    double dist[LIDAR_BUFFER_SIZE];
    int quality[LIDAR_BUFFER_SIZE];

    bool obst_map[GRID_X_SIZE][GRID_Y_SIZE];

    exlcm::obst_map obst_map_msg;


        lidar_raw_handler()
        {
            count = 0;
        }

        ~lidar_raw_handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const exlcm::lidar_raw* msg);

};

void lidar_raw_handler::handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const exlcm::lidar_raw* msg)
{
    printf("Received message on channel \"%s\":\n", chan.c_str());
    printf("  timestamp   = %lld\n", (long long)msg->timestamp);
    printf("  count    = %ld\n", msg->count);
    printf("  enabled     = %d\n", msg->enabled);

    // copy in data
    count = int(msg->count);
    if(count > LIDAR_BUFFER_SIZE)
        count = LIDAR_BUFFER_SIZE;
    for(int i=0; i<count; i++)
    {
        angle[i] = msg->angle[i];
        dist[i] = msg->dist[i];
        quality[i] = msg->quality[i];
    }

 
    // opencv vis
    cv::Mat lidar_image(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255)); // white background

    // draw obstacles
    for (int i = 0; i < count; ++i) 
    {
        if(quality[i] > 40)
        {
            // if it's good quality
            double tmp_x = (dist[i]*cos(angle[i]/180.0*M_PI));
            double tmp_y = (dist[i]*sin(angle[i]/180.0*M_PI));
            int _i_x = int(tmp_x/12000.0*500.0 + 1000.0/2);
            int _i_y = int(tmp_y/12000.0*500.0 + 1000.0/2);

            //std::cout<< _i_x <<" "<<_i_y<<std::endl;

            cv::circle(lidar_image, cv::Point(_i_x, _i_y), 4, cv::Scalar(0, 0, 255), -1);

        }
    }

    // Display the obstacle map
    cv::imshow("Lidar raw", lidar_image);
    cv::waitKey(1); // Update window
}

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    std::cout<<"lcm init done\n"<<std::endl;

    lidar_raw_handler *handlerObject = new lidar_raw_handler();
    lcm.subscribe("LIDAR_RAW", &lidar_raw_handler::handleMessage, handlerObject);

    while(0 == lcm.handle())
    {
        //lcm.publish("OBST_MAP", &handlerObject->obst_map_msg);
        std::cout<<"lcm obst map msg sent"<<std::endl;
    };

    return 0;
}