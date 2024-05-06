#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
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

#define WINDOW_SIZE 800
#define CELL_SIZE 800.0/200.0


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

// function to visualize the obstacle map
void visualize_obst_map(bool *obst_map) {
    // create a window for visualization
    cv::Mat map_image(WINDOW_SIZE, WINDOW_SIZE, CV_8UC3, cv::Scalar(255, 255, 255)); // white background

    // draw obstacles
    for (int i = 0; i < GRID_X_SIZE; ++i) {
        for (int j = 0; j < GRID_Y_SIZE; ++j) {
            if (obst_map[i * GRID_X_SIZE + j]  == 1) {
                // calculate the position of the cell in the visualization window
                int x1 = j * CELL_SIZE;
                int y1 = i * CELL_SIZE;
                int x2 = x1 + CELL_SIZE;
                int y2 = y1 + CELL_SIZE;
                // draw a filled rectangle for the obstacle
                cv::rectangle(map_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 0), -1);
            }
        }
    }

    // Display the obstacle map
    cv::imshow("Obstacle Map", map_image);
    cv::waitKey(1); // Update window
}



int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    std::cout<<"lcm init done\n"<<std::endl;

    obst_map_handler *handlerObject = new obst_map_handler();
    lcm.subscribe("OBST_MAP", &obst_map_handler::handleMessage, handlerObject);

    while(0 == lcm.handle())
    {
        std::cout<<"lidar visualize loop"<<std::endl;
        visualize_obst_map(reinterpret_cast<bool*>(handlerObject->obst_map));
    };

    return 0;
}