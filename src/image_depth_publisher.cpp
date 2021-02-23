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
#include <image_depth_publisher_ros/image_depth_publisher.h>

template<typename MatrixType, typename valuetype>
MatrixType image_depth_publisher::load_csv_matrix (const std::string & path, const char accel ) {
    std::ifstream indata;
    indata.open(path);

    if(!indata.is_open()) {cout<< "Could not open file " << path; };

    std::string line;
    std::vector<valuetype> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, accel)){//',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename MatrixType::Scalar, MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}
image_depth_publisher::image_depth_publisher(ros::NodeHandle nh_) : it(nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    n_p.param<std::string>("image_topic", image_topic, "camera/color/image_raw");
    n_p.param<std::string>("depth_topic", depth_topic, "camera/aligned_depth_to_color/image_raw");
    n_p.param<std::string>("cam_info_topic", cam_info_topic, "camera/color/camera_info");
    n_p.param<std::string>("depth_info_topic", depth_info_topic, "camera/aligned_depth_to_color/camera_info");
    n_p.param<double>("image_rate", image_rate, 30.0);
    n_p.param<std::string>("depth_encoding", depth_encoding, "32FC1");
    n_p.param<std::string>("rgb_encoding", rgb_encoding, "bgr8");


    image_pub = it.advertise(image_topic, 1);
    depth_pub = it.advertise(depth_topic, 1);
    image_info_pub = nh.advertise<sensor_msgs::CameraInfo>(cam_info_topic, 1);
    depth_info_pub = nh.advertise<sensor_msgs::CameraInfo>(depth_info_topic, 1);

    n_p.param<std::string>("depth_image_list", depth_list, "/home/master/intel_ws/Dataset/depthlist.txt");
    n_p.param<std::string>("rgb_image_list", rgb_list, "/home/master/intel_ws/Dataset/rgblist.txt");
    n_p.param<std::string>("dataset_dir", _directory, "/home/master/intel_ws/Dataset");
    n_p.param<std::string>("rgb_intrinsics_file", rgb_intrinsics_list, "/home/master/intel_ws/Dataset/rgb_intrisics.txt");
    n_p.param<std::string>("depth_intrinsics_file", depth_intrinsics_list, "/home/master/intel_ws/Dataset/depth_intrisics.txt");


    Eigen::MatrixXf eig_intrinsics(3,3);
    eig_intrinsics.setZero(3,3);
    eig_intrinsics = load_csv_matrix<Matrix3f, float>(rgb_intrinsics_list);

    cam_info_msg.K[0] = eig_intrinsics(0,0);
    cam_info_msg.K[2] = eig_intrinsics(0,2);
    cam_info_msg.K[4] = eig_intrinsics(1,1);
    cam_info_msg.K[5] = eig_intrinsics(1,2);
    cam_info_msg.K[8] = 1;
    cam_info_msg.D.resize(5);
    cam_info_msg.D[0] = 0;
    cam_info_msg.D[1] = 0;
    cam_info_msg.D[2] = 0;
    cam_info_msg.D[3] = 0;
    cam_info_msg.D[4] = 0;

    eig_intrinsics.setZero(3,3);
    eig_intrinsics = load_csv_matrix<Matrix3f, float>(depth_intrinsics_list);

    depth_info_msg.K[0] = eig_intrinsics(0,0);
    depth_info_msg.K[2] = eig_intrinsics(0,2);
    depth_info_msg.K[4] = eig_intrinsics(1,1);
    depth_info_msg.K[5] = eig_intrinsics(1,2);
    depth_info_msg.K[8] = 1;
    depth_info_msg.D.resize(5);
    depth_info_msg.D[0] = 0;
    depth_info_msg.D[1] = 0;
    depth_info_msg.D[2] = 0;
    depth_info_msg.D[3] = 0;
    depth_info_msg.D[4] = 0;
    frame = 0;
    ROS_INFO("RGB/Depth Publisher Initialized");
}


void image_depth_publisher::run()
{

    static ros::Rate rate(image_rate);
    _depthFile.open(depth_list);
    _rgbFile.open(rgb_list);
    while (ros::ok())
    {


        if(!load_frame())
            break;


        img_msg = cv_bridge::CvImage(std_msgs::Header(), rgb_encoding, currRGB).toImageMsg();
        img_msg->header.seq = frame;
        img_msg->header.stamp = ros::Time::now();
        img_msg->header.frame_id = "camera_optical_frame";

        cam_info_msg.height = currRGB.rows;
        cam_info_msg.width = currRGB.cols;

        depth_msg = cv_bridge::CvImage(std_msgs::Header(), depth_encoding, currDepth).toImageMsg();
        depth_msg->header.seq = frame;
        depth_msg->header.stamp = ros::Time::now();
        depth_msg->header.frame_id = "camera_depth_frame";
        depth_info_msg.height = currDepth.rows;
        depth_info_msg.width = currDepth.cols;
        frame++;

        image_pub.publish(*img_msg);
        depth_pub.publish(*depth_msg);
        cam_info_msg.header = img_msg->header;
        depth_info_msg.header = depth_msg->header;
        image_info_pub.publish(cam_info_msg);
        depth_info_pub.publish(depth_info_msg);
        ros::spinOnce();
        rate.sleep();
    }
    cout<<"No more RGB/Depth Images to publish"<<endl;
    cout<<"Exiting.."<<endl;
}



/* tokenize a line and extract values */
void image_depth_publisher::tokenize(const std::string &str, std::vector<std::string> &tokens, std::string delimiters )  {
        tokens.clear();

        std::string::size_type lastPos    = str.find_first_not_of (delimiters, 0);
        std::string::size_type pos        = str.find_first_of     (delimiters, lastPos);

        while (std::string::npos != pos || std::string::npos != lastPos) {
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            lastPos = str.find_first_not_of(delimiters, pos);
            pos = str.find_first_of(delimiters, lastPos);
        }
    }

/* load your frame */
bool image_depth_publisher::load_frame() {
  std::string               currentLine;
  std::vector<std::string>  tokens;
  std::vector<std::string>  timeTokens;

  if(_depthFile.eof() || _rgbFile.eof()) return false;
  do {
    getline(_depthFile, currentLine);
    tokenize(currentLine, tokens);
  } while (tokens.size() > 2);

  if (tokens.size() == 0)
    return 0;

  std::string depthTimeframe  = tokens[0];
  std::string depthToken      = tokens[1];
  std::string depthLoc        = _directory;
  depthLoc.append("/");
  depthLoc.append(tokens[1]);
  
  cout<<"Depth Location "<<depthLoc<<endl;
  currentLine="";
  tokens.clear();
  timeTokens.clear();
  do {
    getline(_rgbFile, currentLine);
    tokenize(currentLine, tokens);
  } while (tokens.size() > 2);

  std::string rgbTimeframe  = tokens[0];
  std::string rgbToken      = tokens[1];
  std::string rgbLoc        = _directory;
  rgbLoc.append("/");
  rgbLoc.append(tokens[1]);
  cout<<"RGB Location "<<rgbLoc<<endl;

  std::cout << "Loading [" << depthTimeframe << "] " << depthToken << " and [" << rgbTimeframe << "] " << rgbToken;

  tokenize(tokens[0], timeTokens, ".");

  std::string timeString = timeTokens[0];
  timeString.append(timeTokens[1]);

  uint64_t time;
  std::istringstream(timeString) >> time;

currRGB = cv::imread(rgbLoc, cv::IMREAD_ANYCOLOR);
currDepth = cv::imread(depthLoc, cv::IMREAD_ANYDEPTH);
return true;
}
