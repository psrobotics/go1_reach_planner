/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 * 
 *  modified 0501 shuang
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <iostream>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

// lcm
#include <lcm/lcm-cpp.hpp>
#include "lidar_raw.hpp"


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

void print_usage(int argc, const char * argv[])
{
    printf("Usage:\n"
           " For serial channel\n %s --channel --serial <com port> [baudrate]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
		   " For udp channel\n %s --channel --udp <ipaddr> [port NO.]\n"
           " The T1 default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
	const char * opt_is_channel = NULL; 
	const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
	sl_u32         opt_channel_param_second = 0;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

	bool useArgcBaudrate = false;

    IChannel* _channel;

    // Init the lcm server 
    // https://lcm-proj.github.io/lcm/content/tutorial-cpp.html
    lcm::LCM lcm;
    if(!lcm.good())
    {
        std::cout << "lcm init failed" << std::endl;
        std::terminate();
    }
    else
        std::cout << "lcm init done!" << std::endl;

    // init a lcm type for lidar raw info passthrough
    exlcm::lidar_raw lidar_raw_msg;


    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", SL_LIDAR_SDK_VERSION);

	 
	if (argc>1)
	{ 
		opt_is_channel = argv[1];
	}
	else
	{
		print_usage(argc, argv);
		return -1;
	}

	if(strcmp(opt_is_channel, "--channel")==0){
		opt_channel = argv[2];
		if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0)
		{
			// read serial port from the command line...
			opt_channel_param_first = argv[3];// or set to a fixed value: e.g. "com3"
			// read baud rate from the command line if specified...
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);	
			useArgcBaudrate = true;
		}
		else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0)
		{
			// read ip addr from the command line...
			opt_channel_param_first = argv[3];//or set to a fixed value: e.g. "192.168.11.2"
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);//e.g. "8089"
			opt_channel_type = CHANNEL_TYPE_UDP;
		}
		else
		{
			print_usage(argc, argv);
			return -1;
		}
	}
	else
	{
		print_usage(argc, argv);
        return -1;
	}

	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
	{
		if (!opt_channel_param_first) {
#ifdef _WIN32
		// use default com port
		opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
		opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
		opt_channel_param_first = "/dev/ttyUSB0";
#endif
		}
	}

    
    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT){
        if(useArgcBaudrate){
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);

                if (SL_IS_OK(op_result)) 
                {
	                connectSuccess = true;
                }
                else{
                    delete drv;
					drv = NULL;
                }
            }
        }
        else{
            size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
			for(size_t i = 0; i < baudRateArraySize; ++i)
			{
				_channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);

                    if (SL_IS_OK(op_result)) 
                    {
	                    connectSuccess = true;
                        break;
                    }
                    else{
                        delete drv;
					    drv = NULL;
                    }
                }
			}
        }
    }
    else if(opt_channel_type == CHANNEL_TYPE_UDP){
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result)) 
            {
	            connectSuccess = true;
            }
            else{
                delete drv;
				drv = NULL;
            }
        }
    }


    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
			(fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
				, opt_channel_param_first));
		
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }



    signal(SIGINT, ctrlc);
    
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
    while (1) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) 
        {
            drv->ascendScanData(nodes, count);
            // fill in lcm data, lcm max buffer = 1500
            lidar_raw_msg.enabled = true;
            lidar_raw_msg.count = count;

            for (int pos = 0; pos < (int)count ; ++pos) 
            {
                lidar_raw_msg.angle[pos] = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                lidar_raw_msg.dist[pos] = nodes[pos].dist_mm_q2/4.0f;
                lidar_raw_msg.quality[pos] = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

                
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    lidar_raw_msg.angle[pos],
                    lidar_raw_msg.dist[pos],
                    lidar_raw_msg.quality[pos]);
                
                //printf("%ld\n",count);
            }

            // send lcm info to lidar raw
            lcm.publish("LIDAR_RAW", &lidar_raw_msg);
            std::cout<<"lcm message published"<<std::endl;
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
	delay(200);
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    // done!
on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}

