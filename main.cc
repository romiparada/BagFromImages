#include<iostream>
#include <iomanip>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "Thirdparty/DLib/FileFunctions.h"
#include <opencv2/imgcodecs/legacy/constants_c.h>


using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BagFromImages");

    if(argc!=5)
    {
        cerr << "Usage: rosrun BagFromImages BagFromImages <path to image directory> <image extension .ext> <frequency> <path to output bag>" << endl;
        return 0;
    }

    ros::start();

    string dir(argv[1]);

    // Vector of paths to image
    vector<string> filenames_rgb =
            DUtils::FileFunctions::Dir((dir + "/rgb").c_str(), argv[2], true);

    // Vector of paths to image
    vector<string> filenames_depth =
            DUtils::FileFunctions::Dir((dir + "/depth").c_str(), argv[2], true);

    //cout << "Images: " << filenames.size() << endl;

    // Frequency
    double freq = atof(argv[3]);

    // Output bag
    rosbag::Bag bag_out(argv[4],rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    const float T=1.0f/freq;
    ros::Duration d(T);


    ros::Time t_rgb = t;
    for(size_t i=0;i<filenames_rgb.size();i++)
    {
        if(!ros::ok())
            break;

        cv::Mat im = cv::imread(filenames_rgb[i],CV_LOAD_IMAGE_COLOR);
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::BGR8;

	int offset = (dir+"/rgb/").size();
	double sec = stod(filenames_rgb[i].substr(offset, 16));
	t_rgb = ros::Time(sec);
	
        cvImage.header.stamp = t_rgb;
        bag_out.write("/rgb/image_raw", t_rgb, cvImage.toImageMsg());

        //t_rgb+=d;

        cout << i << " / " << filenames_rgb.size() << endl;
    }


    ros::Time t_depth = t;
    for(size_t i=0;i<filenames_depth.size();i++)
    {
        if(!ros::ok())
            break;

        cv::Mat im = cv::imread(filenames_depth[i], CV_LOAD_IMAGE_UNCHANGED);
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

	int offset = (dir+"/depth/").size();
	double sec = stod(filenames_depth[i].substr(offset, 16));
	t_depth = ros::Time(sec);

        cvImage.header.stamp = t_depth;
        bag_out.write("/depth/image_raw",ros::Time(t_depth),cvImage.toImageMsg());

        //t_depth+=d;

        cout << i << " / " << filenames_depth.size() << endl;
    }

    bag_out.close();

    ros::shutdown();

    return 0;
}
