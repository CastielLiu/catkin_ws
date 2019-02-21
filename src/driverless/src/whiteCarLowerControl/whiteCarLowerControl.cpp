#include <driverless/Gps.h>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <stdint.h>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class WhiteCar
{
	public:
		WhiteCar(float max_angle)
		{
			MAX_roadWheelAngle = max_angle;
		
		}
		~WhiteCar()
		{
			boost::mutex::scoped_lock look(mutex_);
			if(port_)
			{
				port_->cancel();
				port_->close();
				port_.reset();
			}
			io_service_.stop();
			io_service_.reset();
		}
		
		bool init()
		{
			if(port_)
			{
				ROS_ERROR("error : port is already opened..");
				return false;
			}
			port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
			port_->open(port_name_,ec_);
			if (ec_) 
			{
				ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
				return false;
			}
			// option settings...
			port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
			port_->set_option(boost::asio::serial_port_base::character_size(8));
			port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
			port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
			port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
			return true;
		}
		
		void run()
		{
			ros::NodeHandle nh;
			ros::NodeHandle private_nh("~");
			
			private_nh.param<std::string>("port_name",port_name_,std::string("dev/ttyUSB0"));
			private_nh.param<int>("baud_rate",baud_rate_,115200);
			
			if(init())
			{
				sub = nh.subscribe<driverless::Gps>("/cmd_vel",10,&WhiteCar::twist_callback,this);
				ros::spin();
			}
		}
		
		void twist_callback(driverless::Gps::ConstPtr& msg)
		{
			float speed = msg->speed;//0-1
			float roadWheelAngle = msg->roadWheelAngle;
			unsigned char sendBuf[9] = {0x55,0xAA,0x01,0x04};
			sendBuf[4] = uint16_t(speed*2048)/256;
			sendBuf[5] = uint16_t(speed*2048)%256;
			if(roadWheelAngle > MAX_roadWheelAngle) roadWheelAngle = MAX_roadWheelAngle;
			else if(roadWheelAngle < -MAX_roadWheelAngle) roadWheelAngle = -MAX_roadWheelAngle;
			uint16_t angleValue = 4096 - (MAX_roadWheelAngle - roadWheelAngle)/(2*MAX_roadWheelAngle)*4096;
			sendBuf[6] = angleValue/256;
			sendBuf[7] = angleValue%256;
			sendBuf[8] = generate_check_num(&sendBuf[4],4);
			boost::asio::write(*port_.get(),boost::asio::buffer(sendBuf,9),ec_);
			ROS_INFO("sendBuf");
		}
		
		unsigned char generate_check_num(unsigned char* buf,unsigned char len)
		{
			unsigned char temp = 0x00;
			for(int i=0;i<len;i++)
				temp ^= *(buf+i);
			return temp;
		}
		
		
		
	private:
		ros::Subscriber sub;
		std::string port_name_;
		int baud_rate_;
		boost::system::error_code ec_;
		boost::asio::io_service io_service_;
		serial_port_ptr port_;
		float MAX_roadWheelAngle;
		boost::mutex mutex_;
		

};

int main(int argc,char **argv)
{
	ros::init(argc,argv,"white_car_lowercontrol_node");
	
}
