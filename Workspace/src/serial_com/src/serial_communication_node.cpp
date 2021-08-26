/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
	unsigned char abc[256];

        if(ser.available()){
	    size_t length = 0;
            ROS_INFO_STREAM("Reading from serial port");
	    length = ser.read(abc, ser.available());
	    abc[length] = '\n';
//	    std_msgs::String str = abc;
  	    //std_msgs::String str;
            //std_msgs::String resdult;
            //result.data = ser.read(ser.available());
            std::cout<<  "Read: " << abc[0]<<"  " << abc[1]<<"  "  << abc[2]<<"\r\n  " ;
//ROS_INFO_STREAM();
           // ROS_INFO_STREAM("Read: " << str);
           // ROS_INFO_STREAM("Read: " << str);

  //          ROS_INFO_STREAM("Read: " << str);
	    //for (int i = 0; i<length; ++i) 
            //    ROS_INFO_STREAM(i, abc[i])
          //  read_pub.publish(abc);
        }
        loop_rate.sleep();

    }
}

