/* 
 * Copyright 2021 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *		 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

class image_depth_publisher
{
    /// current image frame
    int frame;
    ///image dimensions
    int height, width;
    /// camera distorsion
    double k1, k2, k3, t1, t2;
    /// camera calibration
    double cx, cy, fx, fy;
    /// ROS nodehanlder
    ros::NodeHandle nh;
    /// ROS image transport for image callback
    image_transport::ImageTransport it;
    /// ROS image subscriber
    image_transport::Publisher image_pub, depth_pub;
    ///placeholders for  current Grayscale Image
    cv::Mat currRGB, currDepth;
    /// camera matrix
    cv::Mat cam_intrinsics; 
    /// ROS image, camera info topics 
    std::string image_topic, depth_topic, cam_info_topic, depth_info_topic, depth_list, rgb_list, _directory, rgb_encoding, depth_encoding, rgb_intrinsics_list, depth_intrinsics_list;
    std::ifstream _depthFile, _rgbFile;

    double image_rate;
    ros::Publisher image_info_pub, depth_info_pub;
    sensor_msgs::ImagePtr img_msg, depth_msg;
    sensor_msgs::CameraInfo cam_info_msg, depth_info_msg;
    void tokenize(const std::string &str, std::vector<std::string> &tokens, std::string delimiters= " ");
    bool load_frame();
    template<typename MatrixType, typename valuetype> MatrixType load_csv_matrix (const std::string & path, const char accel = ' ');
public:
    /** @fn  blurr_detector(ros::NodeHandle nh_);
	 *  @brief Initializes the blurr_detector
     *  @param nh_ ros nodehandler 
	 */
    image_depth_publisher(ros::NodeHandle nh_);
    void run();
};
