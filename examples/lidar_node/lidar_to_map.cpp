// convert lidar to obst map

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


    private:

    double deg_to_rad(double degrees) 
    {
        // Ensure degrees are within [0, 360) range
        double degs_in_range = std::fmod(degrees, 360.0);
        if (degs_in_range < 0)
            degs_in_range += 360.0;

        return degs_in_range * M_PI / 180.0;
    }

    void print_obst_map()
    {
        for (int i = 0; i < GRID_X_SIZE; i+=5) 
        {
            for (int j = 0; j < GRID_Y_SIZE; j+=5) 
            {
                std::cout << obst_map[i][j] << " ";
            }
            std::cout << std::endl;
        }       
    }

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

    // clear obst map
        for (int i = 0; i < GRID_X_SIZE; i++) 
            for (int j = 0; j < GRID_Y_SIZE; j++)
            {
                obst_map[i][j] = 0;
                obst_map_msg.obst_map[i][j] = 0;
            }

    // convert lidar to obst map
    for(int i=0; i<count; i++)
    {
        // current dist
        if(msg->quality[i]>40)
        {
        double _d = 0;
        for (_d = msg->dist[i]; _d<LIDAR_DIST_MAX; _d+=CONVERT_STEP)
        {
            double _rad_n = deg_to_rad(msg->angle[i]);
            double _x = _d * cos(_rad_n);
            double _y = _d * sin(_rad_n);

            int _x_i = int(_x/1000.0/GRID_X_M * GRID_X_SIZE + GRID_X_SIZE/2);
            int _y_i = int(_y/1000.0/GRID_Y_M * GRID_Y_SIZE + GRID_Y_SIZE/2);

            if(_x_i < 0)
                _x_i = 0;
            else if(_x_i > GRID_X_SIZE - 1)
                _x_i = GRID_X_SIZE - 1;

            if(_y_i < 0)
                _y_i = 0;
            else if(_y_i > GRID_Y_SIZE - 1)
                _y_i = GRID_Y_SIZE - 1;

            obst_map[_x_i][_y_i] = 1;
            obst_map_msg.obst_map[_x_i][_y_i] = 1;
        }
        }
    }

    // send lcm msg
    obst_map_msg.size_x = GRID_X_M;
    obst_map_msg.size_y = GRID_Y_M;
    obst_map_msg.timestamp = 0;
    obst_map_msg.enabled = true;

    // just for debug
    //print_obst_map();
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
        lcm.publish("OBST_MAP", &handlerObject->obst_map_msg);
        std::cout<<"lcm obst map msg sent"<<std::endl;
    };

    return 0;
}