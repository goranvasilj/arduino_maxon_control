/***
 * Modbus master for communicating over serial port
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#define DEFAULT_SERIAL_PORT "/dev/ttyACM0"
#define DEFAULT_BAUD_RATE 460800
#include <chrono>
#include <thread>



serial::Serial ser;
uint8_t data1[500];
double received_data[13];
float command=0;
int received_command=0;

void command_callback(const std_msgs::Float32::ConstPtr &msg) {
	float data=msg->data;
	command=data;
	received_command=1;
	printf("%.2f\n",data);
	
}

double GetDoubleFromString(std::string data, int start, int end)
{
	double value=0;
	double pot=1.;
	int decimal=0;
	for (int i=start;i<=end;i++)
	{
		if (data[i]=='-')
		{
			continue;
		}
		if (data[i]=='.') {
			decimal=1;
			continue;
		}
		if (decimal==1)
		{
			pot=pot*10;
		}
		value = value*10 + data[i]-48;
	}
	value=value/pot;
	if (data[start]=='-')
	{
		value=value*(-1.);
	}
	return value;
}
double Parse(const char* data,int count)
{
	int start=6;
	int data_count=1;
	received_data[0]=(data[2]-48.)*100+(data[3]-48.)*10+(data[4]-48.)*1;
	
	if (data[1]=='N') received_data[0]=-received_data[0];
	
	for (int i = 7; i < count; i++)
	{
		if (data[i] == ' ' || data[i] == '\n'|| data[i] == 13)
		{
			received_data[data_count]=GetDoubleFromString(data,start,i-1);
			start=i+1;
			data_count=data_count+1;
			if (data_count>=3) break;
		}
	}
	return 0.;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "motor_control_arduino");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    std::string port, cmd_topic,state;
    int baudrate,offset;
    nh_ns.param("port", port, (std::string) DEFAULT_SERIAL_PORT); 
    nh_ns.param("baudrate", baudrate, DEFAULT_BAUD_RATE);
    nh_ns.param("motor_cmd", cmd_topic, (std::string) "/motor_cmd");
    nh_ns.param("motor_state", state , (std::string) "/motor_state");

    ros::Publisher state_pub = nh.advertise<sensor_msgs::JointState>(state, 1000);

   //open port
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(30);
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

    ros::Rate loop_rate(500);
    bool first = true;
    double start_stamp=0;
    std::string data_start;
    int count = ser.readline(data_start,500,(std::string)"\n");



    ros::Subscriber motor_cmd_subscriber = nh.subscribe(cmd_topic, 1,
			command_callback);
    ros::spinOnce();
    
    sensor_msgs::JointState sensor_state;
    sensor_state.name.push_back("drill_motor");
    sensor_state.position.push_back(0);
    sensor_state.velocity.push_back(0);
    sensor_state.effort.push_back(0);
    command=0;
    received_command=1;
    while(ros::ok()){

        ros::spinOnce();
        std::string data2;
        int count = ser.readline(data2,500,(std::string)"\n");
        std::cout<<data2<<std::endl;

        if (count>10 && (data2[0]=='E' || data2[0]=='D'))
        {
        	double number=Parse(data2.c_str(),count);
		sensor_state.position[0]=received_data[0];
		sensor_state.velocity[0]=received_data[1];
		sensor_state.effort[0]=received_data[2];
		state_pub.publish(sensor_state);
        }
        if (ser.available()<500)
        {
        	loop_rate.sleep();
        }else
        {
            ros::spinOnce();
        }
        
	if (received_command==1)
	{
		received_command=0;
		float data=command;
		if (fabs(data)<0.1)
		{
			ser.write("D");
			printf("D\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		} else
		{
			if (data<0)
			{
				ser.write("N");
			printf("N\n");				
			} else
			if (data>0)
			{
				ser.write("P");
			printf("P\n");				
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			data=fabs(data);
			int d=(int) data;
			if (d>200) d=200;
			std::string str="S"+std::to_string(d/100)+std::to_string(d%100/10)+std::to_string(d%10);
			std::cout<<str<<std::endl;
			ser.write(str);
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			
			ser.write("E");
			printf("E\n");				
			
		}
	}
        
    }
}

