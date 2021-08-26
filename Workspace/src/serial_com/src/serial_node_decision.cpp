//* **
// * This example expects the serial port has a loopback on it.
// *
// * Alternatively, you could use an Arduino:
// *
// * <pre>
// *  void setup() {
// *    Serial.begin(<insert your baudrate here>);
// *  }
// *
// *  void loop() {
// *    if (Serial.available()) {
// *      Serial.write(Serial.read());
// *    }
// *  }
// * </pre>
// */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

serial::Serial ser;
ros::Publisher pub_v2x_decision;
std_msgs::Bool result;
unsigned char count; 

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
    ros::Publisher pub_v2x_decision =nh.advertise<std_msgs::Bool>("v2x_decision", 1);
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
    count = 0;

    while(ros::ok()){

        ros::spinOnce();

	    uint8_t inputString[200];
	    unsigned int inputnumber = 0;
            uint8_t straight_signal = 0;
            uint8_t straight_remain_time = 0; 
            result.data = false;

        if(inputnumber = ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            //std_msgs::String result;
            //result.data = ser.read(ser.available());
            //ROS_INFO_STREAM("Read: " << result.data);
            //read_pub.publish(result);
	    ser.read(inputString, inputnumber);
	    ROS_INFO("Service ID = 0x%.4x", (unsigned int)((unsigned int)inputString[4] | ((unsigned int)(inputString[5] << 8))));
	    // S1


          if (inputString[4] == 0x81 && inputString[5] == 0x01)
	    {
             if (count <= 3) count++;
	    }


	    if (inputString[4] == 0x00 && inputString[5] == 0x01)
	    {
		if (inputString[6] == 0) ROS_INFO("CITS Signal OFF");
		else ROS_INFO("CITS Signal ON");

	    }
	    // S2
	    if (inputString[4] == 0x00 && inputString[5] == 0x02)
	    {
		ROS_INFO("My Latitude = %f", (double)((inputString[11] | (inputString[12] << 8) | (inputString[13] << 16) | (inputString[14] << 24))) * 0.0000001);
		ROS_INFO("My Longitude = %f", (double)((inputString[15] | (inputString[16] << 8) | (inputString[17] << 16) | (inputString[18] << 24))) * 0.0000001);
		ROS_INFO("My Elevation = %f", (double)((inputString[19] | (inputString[20] << 8))) * 0.1);
		ROS_INFO("My Direction = %d", (inputString[21] | (inputString[22] << 8)));
		ROS_INFO("Info Count = %d", inputString[23]);
		ROS_INFO("Event ID = %d", (inputString[24] | (inputString[25] << 8)));
		ROS_INFO("Duration = %d", inputString[26]);
		ROS_INFO("Event Latitude = %f", (double)((inputString[27] | (inputString[28] << 8) | (inputString[29] << 16) | (inputString[30] << 24))) * 0.0000001);
		ROS_INFO("Event Longitude = %f", (double)((inputString[31] | (inputString[32] << 8) | (inputString[33] << 16) | (inputString[34] << 24))) * 0.0000001);
		ROS_INFO("Event Elevation = %f", (double)((inputString[35] | (inputString[36] << 8))) * 0.1);
		ROS_INFO("Event Direction = %d", (inputString[37] | (inputString[38] << 8)));
		ROS_INFO("Region Offset = %d", inputString[39]);
		// ...

	    }
	    // S7
	    if (inputString[4] == 0x00 && inputString[5] == 0x07)
	    {
		ROS_INFO("Message ID = %d", (inputString[6] | (inputString[7] << 8)));
		ROS_INFO("Alert ID = %d", inputString[10]);
		ROS_INFO("Event ID = %d", (inputString[11] | (inputString[12] << 8)));
		ROS_INFO("Distance = %d", (inputString[13] | (inputString[14] << 8)));
		ROS_INFO("My Latitude = %f", (double)((inputString[15] | (inputString[16] << 8) | (inputString[17] << 16) | (inputString[18] << 24))) * 0.0000001);
		ROS_INFO("My Longitude = %f", (double)((inputString[19] | (inputString[20] << 8) | (inputString[21] << 16) | (inputString[22] << 24))) * 0.0000001);
		ROS_INFO("My Elevation = %f", (double)((inputString[23] | (inputString[24] << 8))) * 0.1);
		ROS_INFO("My Direction = %d", (inputString[25] | (inputString[26] << 8)));
		ROS_INFO("Event Latitude = %f", (double)((inputString[27] | (inputString[28] << 8) | (inputString[29] << 16) | (inputString[30] << 24))) * 0.0000001);
		ROS_INFO("Event Longitude = %f", (double)((inputString[31] | (inputString[32] << 8) | (inputString[33] << 16) | (inputString[34] << 24))) * 0.0000001);
		ROS_INFO("Event Elevation = %f", (double)((inputString[35] | (inputString[36] << 8))) * 0.1);
		ROS_INFO("Event Direction = %d", (inputString[37] | (inputString[38] << 8)));
		ROS_INFO("Straight Exist = %d", inputString[39]);
		ROS_INFO("Straight Signal Status = %d", inputString[40]);
		ROS_INFO("Straight Signal Remain Sec = %d", inputString[41]);
		ROS_INFO("Left Exist = %d", inputString[42]);
		ROS_INFO("Left Signal Status = %d", inputString[43]);
		ROS_INFO("Left Signal Remain Sec = %d", inputString[44]);
		ROS_INFO("Extra Exist = %d", inputString[45]);
		ROS_INFO("Extra Signal Status = %d", inputString[46]);
		ROS_INFO("Extra Signal Remain Sec = %d", inputString[47]);
		
		straight_signal = inputString[40];
		straight_remain_time = inputString[41];

		if ((straight_signal == 1 || (straight_signal ==3 && straight_remain_time < 7) || straight_remain_time == 2 ))  {
			result.data = false;
		}
                else {
			result.data = true ;
		}


           }
        }
	if (count <= 3) pub_v2x_decision.publish(result);
	else
	{
		result.data = true;
		pub_v2x_decision.publish(result);
	}
        loop_rate.sleep();		
    }
}

