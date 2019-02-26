#include "gng_compressor.h"

#define ECHAN_V 4 // vertical sampling
#define ECHAN_H 8 // horizontal sampling


using namespace gng_imgcompressor;

GngCompressor::GngCompressor(ros::NodeHandle &nh)
{
	
	/* Node handle that manage the node and provide publishing and subscribing function */
    nh_ = nh;

    /* Set up the publisher for the cmd_vel topic : we'll publish on this topic to drive the robot */
    //compressed_pub_ = nh_.advertise</*what*/>("/gng_data_compressed", 1);

	camera_sub_ = nh_.subscribe("/input_image", 1, &GngCompressor::msgCallback, this);
  //  compressed_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/gng_data_compressed", 1);
    compressed_pub_ = nh_.advertise<sensor_msgs::Image>("/gng_data_compressed", 1);
	
}


/* msgCallback :
 *  Callback function called when data is received from Camera ... 
 *  Compute the compressed image
 * */


/*
 * sensor_msgs/CompressedImage.msg
 * 
 * # This message contains a compressed image
 * 
 * Header header        # Header timestamp should be acquisition time of image
 *                      # Header frame_id should be optical frame of camera
 *                      # origin of frame should be optical center of cameara
 *                      # +x should point to the right in the image
 *                      # +y should point down in the image
 *                      # +z should point into to plane of the image
 * string format        # Specifies the format of the data
 *                      #   Acceptable values:
 *                      #     jpeg, png
 * uint8[] data         # Compressed image buffer
*/
/*
sensor_msgs/Image.msg
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in src/image_encodings.cpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
* */

//void GngCompressor::msgCallback(const sensor_msgs::CompressedImage & msg)
void GngCompressor::msgCallback(const sensor_msgs::Image & msg)
{
	//sensor_msgs::CompressedImage processedImage;
	//		sensor_msgs::Image processedImage;
	sensor_msgs::Image tempImage;
	int buffer;
	float mean_R = 0, mean_G = 0, mean_B = 0;

//	ROS_WARN("Image received !");

//	std::cout << "Size :" << msg.height << "x" << msg.width << " endianness : " << msg.is_bigendian << std::endl;
	tempImage.header = msg.header;

	tempImage.height = msg.height / 3;
	tempImage.width = msg.width;

	tempImage.encoding = msg.encoding;
	tempImage.is_bigendian = msg.is_bigendian;
	tempImage.step = msg.step;
	
	tempImage.data = msg.data;
	
	//--------------------------------------	
	/*processedImage.height = msg.height / 2;
	processedImage.width = msg.width;

	processedImage.encoding = msg.encoding;
	processedImage.is_bigendian = msg.is_bigendian;
	processedImage.step = msg.step;
	
	processedImage.data = msg.data;
		
	processedImage.encoding = msg.encoding;
	processedImage.is_bigendian = msg.is_bigendian;
	processedImage.step = msg.step;
	
	processedImage.data = msg.data;*/
	
//	std::cout << "Size :" << msg.height << "x" << msg.width << std::endl;
//	std::cout << "Size : " << tempImage.height << "x" << tempImage.width << std::endl;
	//std::cout << "Size : " << processedImage.height << " " << processedImage.width << std::endl;
	
/*	for(int i = 0 ; i < msg.height * msg.width ; i++)
	{
		if(i < 153600){ tempImage.data[i] = msg.data[i]; }
		else{ tempImage.data[i] = 0; }
	}*/

/*	for(int i = 0 ; i < tempImage.width * tempImage.height ; i++)
	{
		//buffer[i] += msg.data[i];
		//if( (i % 256) == 0 )
		//{
		tempImage.data[i] = 0;
		//buffer=0;
		//}
	}*/
	
/*
 *  ENDIANNESS !
 * 	B G R B G R B G R ...
 *  B G R B G R B G R ...
 *  B G R B G R B G R ...
*/
	
	// for(int j = 0 ; j < 3*tempImage.height ; j++)
	for(int j = 0 ; j < tempImage.height ; j++)
	{
		
		for(int i = 0 ; i < tempImage.width ; i++)
		{
			tempImage.data[3*(j*tempImage.width+i)] = msg.data[3*(j*tempImage.width+i)] ;
			tempImage.data[3*(j*tempImage.width+i)+1] = msg.data[3*(j*tempImage.width+i)+1];
			tempImage.data[3*(j*tempImage.width+i)+2] = msg.data[3*(j*tempImage.width+i)+2];
		}
	}

	for(int k = 0 ; k < ECHAN_V ; k++)
	{
		for(int l = 0 ; l < ECHAN_H ; l++)
		{
			for(int j = k * tempImage.height / ECHAN_V ; j < (k+1)*tempImage.height / ECHAN_V ; j++)
			{
				for(int i = l * tempImage.width / ECHAN_H ; i < (l+1)*tempImage.width / ECHAN_H ; i++)
				{
					mean_B += tempImage.data[3*(j*tempImage.width+i)];
					mean_G += tempImage.data[3*(j*tempImage.width+i)+1];
					mean_R += tempImage.data[3*(j*tempImage.width+i)+2];
					//tempImage.data[3*(j*tempImage.width+i)] = (ECHAN_H-l)*32-1;
					//tempImage.data[3*(j*tempImage.width+i)+1] = 0;
					//tempImage.data[3*(j*tempImage.width+i)+2] = (ECHAN_V-k)*64-1;
				}
			}
			
			mean_B /= tempImage.width / ECHAN_H  * tempImage.height / ECHAN_V;  
			mean_G /= tempImage.width / ECHAN_H  * tempImage.height / ECHAN_V;  
			mean_R /= tempImage.width / ECHAN_H  * tempImage.height / ECHAN_V;  
			for(int j = k * tempImage.height / ECHAN_V ; j < (k+1)*tempImage.height / ECHAN_V ; j++)
			{
				for(int i = l * tempImage.width / ECHAN_H ; i < (l+1)*tempImage.width / ECHAN_H ; i++)
				{
					tempImage.data[3*(j*tempImage.width+i)] = mean_B;
					tempImage.data[3*(j*tempImage.width+i)+1] = mean_G;
					tempImage.data[3*(j*tempImage.width+i)+2] = mean_R;
				}
			}
	
	
		}
	}
	
	compressed_pub_.publish(tempImage);
//	std::cout << "published" <<std::endl;

}

/* ######################################################  */


bool GngCompressor::compress()
{
	// Wait for callback from cam to be called.
    while(nh_.ok())
    {
		ros::spinOnce();
    }
    
    return true;
}


/* ######################################################  */

		
int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "gng_compressor");
  ros::NodeHandle nh;
  
  GngCompressor gng_comp(nh);
  gng_comp.compress();

return 0;
}
