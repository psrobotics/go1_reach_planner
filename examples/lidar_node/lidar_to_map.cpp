// convert lidar to obst map

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <lcm/lcm-cpp.hpp>
#include "lidar_raw.hpp"


class lidar_raw_handler 
{
    public:

    // lidar data array
    int count;
    double angle[1500];
    double dist[1500];
    int quality[1500];


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
    for(int i=0; i<count; i++)
    {
        angle[i] = msg->angle[i];
        dist[i] = msg->dist[i];
        quality[i] = msg->quality[i];
    }
}

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    std::cout<<"lcm init done\n"<<std::endl;

    lidar_raw_handler handlerObject;
    lcm.subscribe("LIDAR_RAW", &lidar_raw_handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}