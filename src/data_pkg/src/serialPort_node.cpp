#include "serial_helper.h"

// Specify a timeout value (in milliseconds). If timeout is zero, block infinitely
static size_t g_timeout_milliseconds = 500;
static size_t g_imuBufferSize;
static size_t g_loopCount;

static my_serials_t my_serials;

void *imu_thread(void *thread_data)
{ 
    my_serials_t *my_serials = (my_serials_t *)thread_data;

#if DEBUG > 0
    cout << INFO(imu_thread, "Started\n");
#endif
    my_serials->log_ofs << INFO(imu_thread, "Started\n");
    while (ros::ok())
    {
        std::string imuWriteData;
        
        my_serials->imu_port->ReadLine(my_serials->imuReadData, 0x0D, 0);
        if(g_imuBufferSize == 0)
        {
            g_imuBufferSize = my_serials->imuReadData.length();
        }
        /* Copy original read data to another for writing */
        imuWriteData = my_serials->imuReadData; 
#if DEBUG > 2
        cout << my_serials->imuReadData;
#endif
        /* Acquired lock to write data to USB port */
        pthread_mutex_lock(&my_serials->usb_lockWrite);
        my_serials->usb_port->Write(imuWriteData);
        pthread_mutex_unlock(&my_serials->usb_lockWrite);
    }

#if DEBUG > 0
    cout << INFO(imu_thread, "Stopped\n");
#endif
    my_serials->log_ofs << INFO(imu_thread, "Stopped\n");
    return 0;
}

void *usb_thread(void *thread_data)
{
    my_serials_t *my_serials = (my_serials_t *)thread_data;
    std::vector<coordinate_ns::mapData_t> mapCoordinatesList;
    size_t mapPreIndex = -1;
    int rc;

#if DEBUG > 0
    cout << INFO(usb_thread, "Started\n");
#endif
    my_serials->log_ofs << INFO(usb_thread, "Started\n");

    while (ros::ok())
    {
	    std::vector<std::string> usbReadDataVector;
	    my_serials->usb_port->ReadLine(my_serials->usbReadData, '\n', 0);
	    usbReadDataVector = split(my_serials->usbReadData, ',');
	#if DEBUG > 2
	    cout << my_serials->usbReadData;
	#endif
	    my_serials->log_ofs << INFO(usb_thread, my_serials->usbReadData);

	    if(usbReadDataVector[0].compare("$VPLAN") == 0)
	    {
		if(usbReadDataVector[1].compare("STOP") == 0)
		{
		    std::stringstream    ss;
		    std::ofstream        map_ofs("map_out_3103");
		    nh::json j = mapCoordinatesList;
		    
		    ss << std::setw(4) << j;
		    if (map_ofs.is_open())
		    {
		        map_ofs << ss.str() << std::endl;
		        map_ofs.close();
		    }
		    else
		        cout << "Error: unable to open file map_out" << endl;

		    if (my_serials->imu_thread_id != 0)
			continue;

		    if ((rc = pthread_create(&my_serials->imu_thread_id, NULL, 
		                            imu_thread, (void *)my_serials)) != 0)
		    {
		        cout << "Error: Unable to create IMU thread, return " << rc << '\n';
		        ros::shutdown();
		    }
		    else 
		        cout << "[usb_thread] Created imu thread with id: " << my_serials->imu_thread_id << endl;
		}
		else if(usbReadDataVector[1].compare("SPLINE") == 0)
		{
		    /* TODO: Handle vehicle message "$VPLAN,SPLINE,<data>,CC\r\n" */
		    mapCoordinatesList.clear();
		}
		else 
		{
		    size_t index = std::stod(usbReadDataVector[1]);
		    if (index == mapPreIndex)
			continue;
		    double x = std::stod(usbReadDataVector[2]);
		    double y = std::stod(usbReadDataVector[3]);
		    mapCoordinatesList.push_back( coordinate_ns::mapData{index, x, y} );
		    mapPreIndex = index;
		}
	    }
	    else if(usbReadDataVector[0].compare("$VDATA") == 0)
	    {
		std_msgs::Float64MultiArray avoidObstacleMsg;
		avoidObstacleMsg.data.push_back(std::stod(usbReadDataVector[1])); // current position X
		avoidObstacleMsg.data.push_back(std::stod(usbReadDataVector[2])); // current position y
		avoidObstacleMsg.data.push_back(std::stod(usbReadDataVector[3])); // heading angle
		avoidObstacleMsg.data.push_back(std::stod(usbReadDataVector[4])); // point's index
		my_serials->pub.publish(avoidObstacleMsg);
	    }
	    else 
	    {
		my_serials->log_ofs << INFO(usb_thread, "unknown type: " << usbReadDataVector[0]);
		continue;
	    }
    }
    
#if DEBUG > 0
    cout << INFO(usb_thread, "Stopped\n");
#endif
    my_serials->log_ofs << INFO(usb_thread, "Stopped\n");
    return 0;
}

void avoidObstacle_callback(const std_msgs::Float64MultiArray &msg)
{
    std::vector<double> myData = msg.data;
    std::string pcDataWrite;

    if(myData.size() == 1)
    {
        std::string mess = "PCDAT,0,";
        pcDataWrite = formatMessage(mess);
    }
    else
    {
        std::stringstream ss_mess;
        ss_mess << "PCDAT,1," << fixed << myData[0] << "," << 
            fixed << myData[1] << ",";
        pcDataWrite = formatMessage(ss_mess.str());
    }
    my_serials.log_ofs << INFO(avoidObstacle_callback, pcDataWrite);
#if DEBUG > 2
    cout << INFO(avoidObstacle_callback, pcDataWrite);
#endif

    pthread_mutex_lock(&my_serials.usb_lockWrite);
    my_serials.usb_port->Write(pcDataWrite);
    pthread_mutex_unlock(&my_serials.usb_lockWrite);
}

/**
 * @brief This node demonstrates reading and writing data from/to
 *        stm32f4xx.
 */
int main(int argc, char **argv)
{
    int rc;
    // System starting time
    int64_t startTime_s = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();

    // Announce this program to the ROS master as a "node" called "serialPort_node"
    ros::init(argc, argv, "serialPort_node");
    ros::NodeHandle nh;

    std::stringstream logName;
    logName << startTime_s << ".log";

    // Instantiate my_serials object.
    my_serials.pub = nh.advertise<std_msgs::Float64MultiArray>("VDATA", 10);
    my_serials.sub = nh.subscribe("PCDAT", 10, avoidObstacle_callback);
    my_serials.usb_port = new SerialPort();
    my_serials.imu_port = new SerialPort();
    if ( pthread_mutex_init(&my_serials.usb_lockWrite, NULL) != 0 ||
         pthread_mutex_init(&my_serials.usb_lockRead, NULL) != 0 ) 
    { 
        cout << "mutex initiation has failed\n"; 
        return 1; 
    } 

    my_serials.log_ofs.open(logName.str(), ofstream::out | ofstream::app); 

    try
    {
        my_serials.usb_port->Open(USB_SERIAL_PORT) ;
        my_serials.imu_port->Open(IMU_SERIAL_PORT) ;
    }
    catch (const OpenFailed&)
    {
        std::cerr << "The serial ports did not open correctly.\n";
        return EXIT_FAILURE ;
    }

    config_serial(my_serials.usb_port, BaudRate::BAUD_115200, CharacterSize::CHAR_SIZE_8,
         FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_NONE, StopBits::STOP_BITS_1);
    config_serial(my_serials.imu_port, BaudRate::BAUD_921600, CharacterSize::CHAR_SIZE_8,
         FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_NONE, StopBits::STOP_BITS_1);

    if ((rc = pthread_create(&my_serials.usb_thread_id, NULL, usb_thread, (void *)&my_serials)) != 0)
    {
        cout << "Error: Unable to create USB thread, return " << rc << '\n';
        return rc;
    }
    else 
        cout << "[main] Created USB thread with id: " << my_serials.usb_thread_id << '\n';
    
    ros::Rate loop_rate(20); // check callback for every 50ms
    cout << "[main] Entering while loop at " << hmsCurrent() << endl;
    while (ros::ok())
    {
        g_loopCount++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
#ifdef DEBUG
    cout << INFO(main, "exited while loop, count=" << g_loopCount << '\n') ;
    my_serials.log_ofs << INFO(main, "exited while loop, count=" << g_loopCount << '\n');
#endif

    // Cancel thread and Join thread to terminate
    if(my_serials.imu_thread_id != 0) 
    {
        pthread_cancel(my_serials.imu_thread_id);
        cout << INFO(main, "Joining IMU thread\n");
        pthread_join(my_serials.imu_thread_id, NULL);
    }
    if(my_serials.usb_thread_id != 0) 
    {
        pthread_cancel(my_serials.usb_thread_id);
        cout << INFO(main, "Joining USB thread\n");
        pthread_join(my_serials.usb_thread_id, NULL);
    }
    // Destroy mutex clock
    pthread_mutex_destroy( &(my_serials.usb_lockWrite) );
    pthread_mutex_destroy( &(my_serials.usb_lockRead) );
    // Close serial ports and deallocating
    my_serials.usb_port->Close() ;
    my_serials.imu_port->Close() ;
    delete my_serials.usb_port;
    delete my_serials.imu_port;
    // Sync file
    my_serials.log_ofs.close();

    // Successful program completion.
    std::cout << INFO(main, "The example program successfully completed!" << endl);
    return EXIT_SUCCESS ;
}
