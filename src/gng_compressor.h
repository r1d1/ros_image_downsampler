#ifndef GNG_COMPRESSOR_H
#define GNG_COMPRESSOR_H

/* Class : GNG Compressor
 * 
 * @author : Erwan Renaudo
 * @created : 20/08/2012
 * @description : This class compresses and extracts data from camera image to feed the GNG
 * 
 * */

#include <iostream>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>


#define PI 3.14159265

namespace gng_imgcompressor {

class GngCompressor
{
	private :
		//! The node handle we'll be using
		ros::NodeHandle nh_;
		//! Pub and Sub
		ros::Publisher compressed_pub_;
		ros::Subscriber camera_sub_;
	
//		geometry_msgs::Twist base_cmd;
//		tf::TransformListener listener_odom_;
		
		std::string target_frame_;
//		float command_duration;
//		float accumulator;
	//	sensor_msgs::Image processedImage;

		
	public :
		GngCompressor(ros::NodeHandle &nh);
		
		bool compress();
	//	void msgCallback(const sensor_msgs::CompressedImage & msg);
		void msgCallback(const sensor_msgs::Image & msg);
//		int signgrad(float * mean, int index, int iterator);
};

}

#endif

