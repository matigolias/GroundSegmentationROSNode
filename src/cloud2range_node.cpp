#include "utils.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "Labeler/label.h"

/////////////////////////////////////
//#include <velodyne_pointcloud/point_types.h>
/////////////////////////////////

#include <math.h>

namespace cloud2range {

using PointT = pcl::PointXYZI;
using cv::Mat;

const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
const int SAME_OUTPUT_TYPE = -1;

class Cloud2RangeNode {
 public:
  explicit Cloud2RangeNode(const ros::NodeHandle& pnh); //explicit impede o compilador de criar conversões, através do construtor, dos parametros das funções da classe

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  Mat RepairGaps(const Mat no_ground_image, int step, float depth_threshold);
  Mat CreateAngleImg(Mat range_image);
  Mat CreateResImage(Mat range_image, Mat smoothed_image);
  // int FindRow(int unprecise_row, float vert_angle);
  Mat GetUniformKernel(int window_size, int type);
  Mat GetSavitskyGolayKernel(int window_size);
  Mat SavitskyGolaySmoothing(const Mat& image, int window_size);
  Mat EraseGroundBFS (Mat range_image, Mat smoothed_image, double ground_angle_threshold, double start_angle_threshold, int kernel_size);
  

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber sub_cloud_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher pub_camera_;

  int n_beams_,rpm_, sample_freq_;
  double min_angle_, max_angle_;
  double min_range_, max_range_;

  //double HDL_64_vert_angle [64]{-0.428567, -0.428567, -0.411010, -0.400830, -0.390212, -0.383550, -0.375084, -0.367629, -0.356546, -0.347179, -0.335927, -0.330268, -0.321645, -0.313528, -0.303135, -0.293236, -0.284032, -0.277615, -0.268517, -0.260519, -0.249417, -0.240953, -0.229741, -0.223919, -0.214571, -0.207927, -0.197139, -0.187686, -0.177605, -0.170069, -0.162315, -0.153144, -0.149445, -0.143444, -0.138635, -0.131811, -0.126182, -0.119537, -0.114497, -0.108036, -0.102780, -0.095696, -0.089616, -0.083935, -0.079062, -0.072558, -0.066659, -0.060551, -0.054439, -0.048731, -0.043427, -0.036488, -0.031588, -0.025460, -0.019944, -0.013200, -0.007886, -0.001549, 0.003561, 0.010307, 0.015620, 0.022568, 0.027267, 0.034210};

  double d_azimuth_, d_altitude_;
  double start_angle_threshold, ground_angle_threshold;
  int n_cols_, window_size;
  
  // We are going to use camera info to store all the above params for decoding
  sensor_msgs::CameraInfo cinfo_;
};

// Cloud2RangeNode::Cloud2RangeNode(const ros::NodeHandle& pnh)
//     : pnh_(pnh), it_(pnh) {
//   sub_cloud_ = pnh_.subscribe("/ns1/velodyne_points", 1, &Cloud2RangeNode::CloudCb, this); ///velodyne_points
//   pub_camera_ = it_.advertiseCamera("range/image", 1);

//   // Read params

//   window_size = pnh_.param("window_size", 5); // 5, 7, 9, 11
//   ROS_ASSERT(window_size == 5 || 7 || 9 || 11);
//   ground_angle_threshold = pnh_.param("ground_angle_threshold", 0.087266);//0.087266); //5º
//   ROS_ASSERT(ground_angle_threshold > 0);
//   start_angle_threshold = pnh_.param("start_angle_threshold", 0.523598); //30º
//   ROS_ASSERT(ground_angle_threshold > 0);

//   n_beams_ = pnh_.param("n_beams", 16);
//   ROS_ASSERT(n_beams_ > 0);
//   rpm_ = pnh_.param("rpm", 600);
//   ROS_ASSERT(rpm_ > 0);
//   sample_freq_ = pnh_.param("sample_freq", 18000);
//   ROS_ASSERT(sample_freq_ > 0);

//   min_angle_ = pnh_.param("min_angle", -0.2617993878);
//   max_angle_ = pnh_.param("max_angle", 0.2617993878);
//   ROS_ASSERT(min_angle_ < max_angle_);

//   min_range_ = pnh_.param("min_range", 0.5);
//   max_range_ = pnh_.param("max_range", 100);
//   ROS_ASSERT(min_range_ < max_range_ && min_range_ >= 0.0);

//   const auto model = pnh_.param<std::string>("model", "VLP-16");
//   ROS_INFO("lidar model: %s", model.c_str());

//   ROS_INFO(
//       "n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], range: [%0.2f, %0.2f]",
//       n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_), min_range_,
//       max_range_);

//   n_cols_ = sample_freq_ * 60 / rpm_;
//   ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

//   d_azimuth_ = Rad_Deg(360.0 / n_cols_);
//   d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
//   ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
//            Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

//   // Fill in cinfo
//   cinfo_.height = n_beams_;
//   cinfo_.width = n_cols_;
//   cinfo_.distortion_model = model;
//   cinfo_.K[0] = min_angle_;
//   cinfo_.K[1] = max_angle_;
//   cinfo_.K[2] = min_range_;
//   cinfo_.K[3] = max_range_;
//   cinfo_.K[4] = d_azimuth_;
//   cinfo_.K[5] = d_altitude_;
// }

// Cloud2RangeNode::Cloud2RangeNode(const ros::NodeHandle& pnh)
//     : pnh_(pnh), it_(pnh) {
//   sub_cloud_ = pnh_.subscribe("/velodyne_points", 1, &Cloud2RangeNode::CloudCb, this);
//   pub_camera_ = it_.advertiseCamera("range/image", 1);

//   // Read params
//   n_beams_ = pnh_.param("n_beams", 32);
//   ROS_ASSERT(n_beams_ > 0);
//   rpm_ = pnh_.param("rpm", 600);
//   ROS_ASSERT(rpm_ > 0);
//   sample_freq_ = pnh_.param("sample_freq", 18000);
//   ROS_ASSERT(sample_freq_ > 0);

//   min_angle_ = pnh_.param("min_angle", -0.535293);
//   max_angle_ = pnh_.param("max_angle", 0.186227);
//   ROS_ASSERT(min_angle_ < max_angle_);

//   min_range_ = pnh_.param("min_range", 0.5);
//   max_range_ = pnh_.param("max_range", 100);
//   ROS_ASSERT(min_range_ < max_range_ && min_range_ >= 0.0);

//   const auto model = pnh_.param<std::string>("model", "HDL-32");
//   ROS_INFO("lidar model: %s", model.c_str());

//   ROS_INFO(
//       "n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], range: [%0.2f, %0.2f]",
//       n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_), min_range_,
//       max_range_);

//   n_cols_ = sample_freq_ * 60 / rpm_;
//   ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

//   d_azimuth_ = Rad_Deg(360.0 / n_cols_);
//   d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
//   ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
//            Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

//   // Fill in cinfo
//   cinfo_.height = n_beams_;
//   cinfo_.width = n_cols_;
//   cinfo_.distortion_model = model;
//   cinfo_.K[0] = min_angle_;
//   cinfo_.K[1] = max_angle_;
//   cinfo_.K[2] = min_range_;
//   cinfo_.K[3] = max_range_;
//   cinfo_.K[4] = d_azimuth_;
//   cinfo_.K[5] = d_altitude_;
// }



// Cloud2RangeNode::Cloud2RangeNode(const ros::NodeHandle& pnh)
//     : pnh_(pnh), it_(pnh) {
//   sub_cloud_ = pnh_.subscribe("/velodyne_points", 1, &Cloud2RangeNode::CloudCb, this); // /kitti/velo/pointcloud
//   pub_camera_ = it_.advertiseCamera("range/image", 1);

//   // Read params
//   window_size = pnh_.param("window_size", 9); // 5, 7, 9, 11
//   ROS_ASSERT(window_size == 5 || 7 || 9 || 11);
//   ground_angle_threshold = pnh_.param("ground_angle_threshold", 0.0872);//0.087266); //5º
//   ROS_ASSERT(ground_angle_threshold > 0);
//   start_angle_threshold = pnh_.param("start_angle_threshold", 0.523598); //30º
//   ROS_ASSERT(ground_angle_threshold > 0);

//   n_beams_ = pnh_.param("n_beams", 64);
//   ROS_ASSERT(n_beams_ > 0);
//   rpm_ = pnh_.param("rpm", 600);
//   ROS_ASSERT(rpm_ > 0);
//   sample_freq_ = pnh_.param("sample_freq", 20000);//18
//   ROS_ASSERT(sample_freq_ > 0);

//   min_angle_ = pnh_.param("min_angle", -0.453785);//-0.428567);//
//   max_angle_ = pnh_.param("max_angle", 0.104719);//0.034210);//
//   ROS_ASSERT(min_angle_ < max_angle_);

//   min_range_ = pnh_.param("min_range", 0.5);
//   max_range_ = pnh_.param("max_range", 100);
//   ROS_ASSERT(min_range_ < max_range_ && min_range_ >= 0.0);

//   const auto model = pnh_.param<std::string>("model", "HDL-64");
//   ROS_INFO("lidar model: %s", model.c_str());

//   ROS_INFO(
//       "n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], range: [%0.2f, %0.2f]",
//       n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_), min_range_,
//       max_range_);

//   n_cols_ = sample_freq_ * 60 / rpm_;
//   ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

//   d_azimuth_ = Rad_Deg(360.0 / n_cols_);
//   d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
//   ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
//            Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

//   // Fill in cinfo
//   cinfo_.height = n_beams_;
//   cinfo_.width = n_cols_;
//   cinfo_.distortion_model = model;
//   cinfo_.K[0] = min_angle_;
//   cinfo_.K[1] = max_angle_;
//   cinfo_.K[2] = min_range_;
//   cinfo_.K[3] = max_range_;
//   cinfo_.K[4] = d_azimuth_;
//   cinfo_.K[5] = d_altitude_;
// }


// Cloud2RangeNode::Cloud2RangeNode(const ros::NodeHandle& pnh)
//     : pnh_(pnh), it_(pnh) {
//   sub_cloud_ = pnh_.subscribe("velodyne_points", 1, &Cloud2RangeNode::CloudCb, this);
//   pub_camera_ = it_.advertiseCamera("range/image", 1);

//   // Read params
//   n_beams_ = pnh_.param("n_beams", 128);
//   ROS_ASSERT(n_beams_ > 0);
//   rpm_ = pnh_.param("rpm", 600);
//   ROS_ASSERT(rpm_ > 0);
//   sample_freq_ = pnh_.param("sample_freq", 18000);
//   ROS_ASSERT(sample_freq_ > 0);

//   min_angle_ = pnh_.param("min_angle", -0.436332);
//   max_angle_ = pnh_.param("max_angle", 0.261799);
//   ROS_ASSERT(min_angle_ < max_angle_);

//   min_range_ = pnh_.param("min_range", 0.5);
//   max_range_ = pnh_.param("max_range", 100);
//   ROS_ASSERT(min_range_ < max_range_ && min_range_ >= 0.0);

//   const auto model = pnh_.param<std::string>("model", "VLS-128");
//   ROS_INFO("lidar model: %s", model.c_str());

//   ROS_INFO(
//       "n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], range: [%0.2f, %0.2f]",
//       n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_), min_range_,
//       max_range_);

//   n_cols_ = sample_freq_ * 60 / rpm_;
//   ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

//   d_azimuth_ = Rad_Deg(360.0 / n_cols_);
//   d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
//   ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
//            Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

//   // Fill in cinfo
//   cinfo_.height = n_beams_;
//   cinfo_.width = n_cols_;
//   cinfo_.distortion_model = model;
//   cinfo_.K[0] = min_angle_;
//   cinfo_.K[1] = max_angle_;
//   cinfo_.K[2] = min_range_;
//   cinfo_.K[3] = max_range_;
//   cinfo_.K[4] = d_azimuth_;
//   cinfo_.K[5] = d_altitude_;
// }


Cloud2RangeNode::Cloud2RangeNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh) {
  sub_cloud_ = pnh_.subscribe("/os_cloud_nodee/points", 1, &Cloud2RangeNode::CloudCb, this);
  pub_camera_ = it_.advertiseCamera("range/image", 1);

  // Read params

  window_size = pnh_.param("window_size", 7); // 5, 7, 9, 11
  ROS_ASSERT(window_size == 5 || 7 || 9 || 11);
  ground_angle_threshold = pnh_.param("ground_angle_threshold", 0.09);//0.087266); //5º
  ROS_ASSERT(ground_angle_threshold > 0);
  start_angle_threshold = pnh_.param("start_angle_threshold", 0.523598); //30º
  ROS_ASSERT(ground_angle_threshold > 0);

  n_beams_ = pnh_.param("n_beams", 64);
  ROS_ASSERT(n_beams_ > 0);
  rpm_ = pnh_.param("rpm", 600);
  ROS_ASSERT(rpm_ > 0);
  sample_freq_ = pnh_.param("sample_freq", 20000); //512  1024  2048
  ROS_ASSERT(sample_freq_ > 0);

  min_angle_ = pnh_.param("min_angle", -0.392699);//-0.7853981634);//
  max_angle_ = pnh_.param("max_angle", 0.392699);//0.7853981634);//
  ROS_ASSERT(min_angle_ < max_angle_);

  min_range_ = pnh_.param("min_range", 0.3);
  max_range_ = pnh_.param("max_range", 120);
  ROS_ASSERT(min_range_ < max_range_ && min_range_ >= 0.0);

  const auto model = pnh_.param<std::string>("model", "OS1-64");
  ROS_INFO("lidar model: %s", model.c_str());

  ROS_INFO(
      "n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], range: [%0.2f, %0.2f]",
      n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_), min_range_,
      max_range_);

  n_cols_ = sample_freq_ * 60 / rpm_;
  ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

  d_azimuth_ = Rad_Deg(360.0 / n_cols_);
  d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
  ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
           Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

  // Fill in cinfo
  cinfo_.height = n_beams_;
  cinfo_.width = n_cols_;
  cinfo_.distortion_model = model;
  cinfo_.K[0] = min_angle_;
  cinfo_.K[1] = max_angle_;
  cinfo_.K[2] = min_range_;
  cinfo_.K[3] = max_range_;
  cinfo_.K[4] = d_azimuth_;
  cinfo_.K[5] = d_altitude_;
}

// Cloud2RangeNode::Cloud2RangeNode(const ros::NodeHandle& pnh)
//     : pnh_(pnh), it_(pnh) {
//   sub_cloud_ = pnh_.subscribe("/os_cloud_node/points", 1, &Cloud2RangeNode::CloudCb, this);
//   pub_camera_ = it_.advertiseCamera("range/image", 1);

//   // Read params
//   n_beams_ = pnh_.param("n_beams", 256);
//   ROS_ASSERT(n_beams_ > 0);
//   rpm_ = pnh_.param("rpm", 600);
//   ROS_ASSERT(rpm_ > 0);
//   sample_freq_ = pnh_.param("sample_freq", 20000); //512  1024  2048
//   ROS_ASSERT(sample_freq_ > 0);

//   min_angle_ = pnh_.param("min_angle", -0.7853981634);
//   max_angle_ = pnh_.param("max_angle", 0.7853981634);
//   ROS_ASSERT(min_angle_ < max_angle_);

//   min_range_ = pnh_.param("min_range", 0.3);
//   max_range_ = pnh_.param("max_range", 100);
//   ROS_ASSERT(min_range_ < max_range_ && min_range_ >= 0.0);

//   const auto model = pnh_.param<std::string>("model", "OS0-128");
//   ROS_INFO("lidar model: %s", model.c_str());

//   ROS_INFO(
//       "n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], range: [%0.2f, %0.2f]",
//       n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_), min_range_,
//       max_range_);

//   n_cols_ = sample_freq_ * 60 / rpm_;
//   ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

//   d_azimuth_ = Rad_Deg(360.0 / n_cols_);
//   d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
//   ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
//            Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

//   // Fill in cinfo
//   cinfo_.height = n_beams_;
//   cinfo_.width = n_cols_;
//   cinfo_.distortion_model = model;
//   cinfo_.K[0] = min_angle_;
//   cinfo_.K[1] = max_angle_;
//   cinfo_.K[2] = min_range_;
//   cinfo_.K[3] = max_range_;
//   cinfo_.K[4] = d_azimuth_;
//   cinfo_.K[5] = d_altitude_;
// }


void Cloud2RangeNode::CloudCb(
  const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // convert to point cloud
  ROS_INFO("ESTOU VIVO 2");
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Convert the PC to the PointXYZIR point type so we can access the ring
  // pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  // pcl::PCLPointCloud2 pcl_pc2; 
  // pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
  // fromPCLPointCloud2(pcl_pc2, *cloud_XYZIR);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Mat range_image = Mat::zeros(n_beams_, n_cols_, CV_16UC1); //CV_16UC1

  //  int row = 0;
  //  float prev_azimuth = 0;
  int r=0, a=0;

  for (size_t i = 0; i < cloud.size(); ++i) { //colocar em função

    //int ring=100;

    // calculate altitude and azimuth and put into range image
    const auto& point = cloud[i];
    const auto range = PointRange(point);

    //ring = cloud_XYZIR->points[i].ring;
    //ROS_INFO("RING -> %d", ring);

    if (range < min_range_ || range > max_range_) {
      // skip invalid range
      r++;     
      continue;
    }
     const auto azimuth = PointAzimuth(point);
     const auto altitude = PointAltitude(point);
     
    if (altitude < min_angle_ || altitude > max_angle_) {
      // skip invalid range
      a++;
      continue;
    }

     //const int row = ring;
     // int round towards zero
     //int row = (altitude - min_angle_) / d_altitude_ + 0.5; //VLP-16 (+0.5)
     
    //  if(azimuth < prev_azimuth && azimuth < 1 && prev_azimuth > 6)
    //    row++;

     //int row = FindRow(row1, altitude);

    int row = ((max_angle_ - altitude) / (max_angle_ - min_angle_)) * n_beams_;

    const int col = static_cast<int>(azimuth / d_azimuth_); //% n_cols_;

    // make sure valid
    //ROS_INFO("AZIMUTH->%f, D_AZIMUTH->%f, N_COLS->%d, COL->%d, ALTITUDE->%f, ROW->%d", azimuth, d_azimuth_, n_cols_, col, altitude, row);
    ROS_ASSERT(row >= 0 && row < n_beams_);
    ROS_ASSERT(col >= 0 && col < n_cols_);

    // normalize range to [0, 1]
    const double range_norm = (range - min_range_) / (max_range_ - min_range_);

    //ROS_INFO("ALTITUDE->%f, MIN_ANGLE->%f, D_ALTITUDE->%f, ROW1->%d, ROW->%d, COL->%d, RANGE_NORM->%f", altitude, min_angle_, d_altitude_, row1, row, col, range_norm* (std::numeric_limits<ushort>::max() - 1) + 1);

    range_image.at<ushort>(row, col) = range_norm * (std::numeric_limits<ushort>::max() - 1) + 1; //(n_beams_-row-1)

    // ROS_INFO("RANGE_NORM -> %f, ROW -> %d, COLUM -> %d", range_norm, row, col);
    // ROS_INFO("RANGE2 -> %d", range_image.at<ushort>(row, col));
    //prev_azimuth = azimuth;
  }

  // ROS_INFO("altitude max -> %f", teste);
  ROS_DEBUG("num points %zu, num pixels %d", cloud.size(),
            cv::countNonZero(range_image));
  ROS_INFO("num points %zu, num pixels %d", cloud.size(),
            cv::countNonZero(range_image));


  ROS_INFO("INV_ALTITUDE -> %d", a);
  ROS_INFO("INV_RANGE -> %d", r);


  Mat repaired_range_image = RepairGaps(range_image, 5, 1.0f); //coolocar estas variáveis no destrutor para não haver memory leakage
  Mat angle_image = CreateAngleImg(repaired_range_image);
  Mat smoothed_image = SavitskyGolaySmoothing(angle_image, window_size);
  //Mat res_image = CreateResImage (range_image, smoothed_image);

  Mat no_ground_image = EraseGroundBFS (range_image, smoothed_image, ground_angle_threshold, start_angle_threshold, window_size);
  ROS_INFO("ESTOU VIVO 1");

  // update header
  cinfo_.header = cloud_msg->header;
  cv_bridge::CvImage cv_image(cloud_msg->header, "mono16", no_ground_image);/////// res_image
  pub_camera_.publish(*cv_image.toImageMsg(), cinfo_);
}

Mat Cloud2RangeNode::RepairGaps(const Mat no_ground_image, int step, float depth_threshold) {

  Mat inpainted_depth = no_ground_image.clone();

  for (int c = 0; c < inpainted_depth.cols; ++c) {
    for (int r = 0; r < inpainted_depth.rows; ++r) {
      float& curr_depth = inpainted_depth.at<float>(r, c);
      if (curr_depth < 0.001f) {
        int counter = 0;
        float sum = 0.0f;
        for (int i = 1; i < step; ++i) {
          if (r - i < 0) {
            continue;
          }
          for (int j = 1; j < step; ++j) {
            if (r + j > inpainted_depth.rows - 1) {
              continue;
            }
            const float& prev = inpainted_depth.at<float>(r - i, c);
            const float& next = inpainted_depth.at<float>(r + j, c);
            if (prev > 0.001f && next > 0.001f && fabs(prev - next) < depth_threshold) {
              sum += prev + next;
              counter += 2;
            }
          }
        }
        if (counter > 0) {
          curr_depth = sum / counter;
        }
      }
    }
  }
  return inpainted_depth;
}

Mat Cloud2RangeNode::CreateAngleImg(Mat range_image)
{
  //Mat angle_image = Mat::zeros(n_beams_, n_cols_, CV_16UC1);
  Mat angle_image = Mat::zeros(range_image.rows, range_image.cols, CV_16UC1);

  int prev_col = 1, prev_row = 0; //prev_col with value !=0
  ushort range_encoded = 0;
  float prev_r = 0, prev_angle_y = 0, angle_y = 0,
  delta_z = 0, delta_x = 0, alfa = 0,  
  vertical_amplitude = max_angle_ - min_angle_;
  double range_norm=0, range=0;

      for (int col=0; col<n_cols_; ++col)
      {
        for (int row=n_beams_-1; row>=0; --row)
        //for (int row=0; row<n_beams_; ++row)
        {
          range_encoded = range_image.at<ushort>(row, col);
          //ROS_INFO("Range Encoded -> %d, ROW -> %d, COL -> %d", range_encoded, row, col);

          if (range_encoded == 0) continue;

          angle_y = row * d_altitude_ - vertical_amplitude/2; 

          range_norm = static_cast<double>(range_encoded - 1) / (std::numeric_limits<ushort>::max() - 1);
          range = range_norm * (max_range_ - min_range_) + min_range_;
          
          if(col==prev_col)
          {
            //angle_y = row * d_altitude_ - vertical_amplitude/2;

            delta_z = abs(prev_r * sin(prev_angle_y) - range * sin(angle_y));//estes senos podem estar em lookup tables
            delta_x = abs(prev_r * cos(prev_angle_y) - range * cos(angle_y));//estes cossenos podem estar em lookup tables
            alfa = atan2(delta_z, delta_x);

            angle_image.at<ushort>(row, col) = static_cast<ushort>((alfa * 100));
            
            //ROS_INFO("ROW -> %d, COL -> %d, RANGE ->%f, ALFA-> %f, ANGLE_Y-> %f, PREV_RANGE-> %f, PREV_ANGLE_Y-> %f, DELTA_Z-> %f, DELTA_X-> %f", row, col, range, alfa, angle_y, prev_r, prev_angle_y, delta_z, delta_x);
          }

          //ROS_INFO("ROW -> %d, COL -> %d, RANGE ->%f, ALFA-> %f, ANGLE_Y-> %f, PREV_RANGE-> %f, PREV_ANGLE_Y-> %f, DELTA_Z-> %f, DELTA_X-> %f", row, col, range, alfa, angle_y, prev_r, prev_angle_y, delta_z, delta_x);

          prev_col = col;
          prev_row = row;
          prev_r = range;
          prev_angle_y = angle_y;

        }
      }

  return angle_image;
}

Mat Cloud2RangeNode::CreateResImage(Mat range_image, Mat smoothed_image)
{
  Mat res_image = Mat::zeros(n_beams_, n_cols_, CV_16UC1);
  //Mat res_image = range_image.clone();

  for (int col=0; col<n_cols_; ++col)
  {
    for (int row=0; row<n_beams_; ++row)
    {
      //ROS_INFO("ALFA-> %d, ROW -> %d, COL -> %d",angle_image.at<ushort>(row, col), row, col);
      if(smoothed_image.at<ushort>(row, col) > 40) //rad * 100
      res_image.at<ushort>(row, col) = range_image.at<ushort>(row, col);
      //ROS_INFO("RES-> %d",res_image.at<ushort>(row, col));
    }
  }

  return res_image;
}

// int Cloud2RangeNode::FindRow(int aprox_row, float vert_angle)//ver igual
// {
//   if (vert_angle < HDL_64_vert_angle[aprox_row])
//   {
//     while (vert_angle < HDL_64_vert_angle[aprox_row])
//     aprox_row--;

//     if(vert_angle < HDL_64_vert_angle[aprox_row] + (HDL_64_vert_angle[aprox_row + 1] - HDL_64_vert_angle[aprox_row]) / 2)
//       return aprox_row;
    
//     else
//       return aprox_row + 1; //row is now precisely calculated
//   }

//   else
//   {
//     while (vert_angle > HDL_64_vert_angle[aprox_row])
//     aprox_row++;

//     if(vert_angle >= HDL_64_vert_angle[aprox_row -1] + (HDL_64_vert_angle[aprox_row] - HDL_64_vert_angle[aprox_row - 1]) / 2)
//       return aprox_row;
    
//     else
//       return aprox_row - 1; //row is now precisely calculated    
//   }
// }

  Mat Cloud2RangeNode::GetUniformKernel(int window_size, int type) { //const {
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  Mat kernel = Mat::zeros(window_size, 1, type);
  kernel.at<float>(0, 0) = 1;
  kernel.at<float>(window_size - 1, 0) = 1;
  kernel /= 2;
  return kernel;
}

  Mat Cloud2RangeNode::GetSavitskyGolayKernel(int window_size) {//const {
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  bool window_size_ok = window_size == 5 || window_size == 7 ||
                        window_size == 9 || window_size == 11;
  if (!window_size_ok) {
    throw std::logic_error("bad window size");
  }
  // below are no magic constants. See Savitsky-golay filter.
  Mat kernel;
  switch (window_size) {
    case 5:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -3.0f;
      kernel.at<float>(0, 1) = 12.0f;
      kernel.at<float>(0, 2) = 17.0f;
      kernel.at<float>(0, 3) = 12.0f;
      kernel.at<float>(0, 4) = -3.0f;
      kernel /= 35.0f;
      return kernel;
    case 7:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -2.0f;
      kernel.at<float>(0, 1) = 3.0f;
      kernel.at<float>(0, 2) = 6.0f;
      kernel.at<float>(0, 3) = 7.0f;
      kernel.at<float>(0, 4) = 6.0f;
      kernel.at<float>(0, 5) = 3.0f;
      kernel.at<float>(0, 6) = -2.0f;
      kernel /= 21.0f;
      return kernel;
    case 9:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -21.0f;
      kernel.at<float>(0, 1) = 14.0f;
      kernel.at<float>(0, 2) = 39.0f;
      kernel.at<float>(0, 3) = 54.0f;
      kernel.at<float>(0, 4) = 59.0f;
      kernel.at<float>(0, 5) = 54.0f;
      kernel.at<float>(0, 6) = 39.0f;
      kernel.at<float>(0, 7) = 14.0f;
      kernel.at<float>(0, 8) = -21.0f;
      kernel /= 231.0f;
      return kernel;
    case 11:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -36.0f;
      kernel.at<float>(0, 1) = 9.0f;
      kernel.at<float>(0, 2) = 44.0f;
      kernel.at<float>(0, 3) = 69.0f;
      kernel.at<float>(0, 4) = 84.0f;
      kernel.at<float>(0, 5) = 89.0f;
      kernel.at<float>(0, 6) = 84.0f;
      kernel.at<float>(0, 7) = 69.0f;
      kernel.at<float>(0, 8) = 44.0f;
      kernel.at<float>(0, 9) = 9.0f;
      kernel.at<float>(0, 10) = -36.0f;
      kernel /= 429.0f;
      return kernel;
  }
  return kernel;
}

  Mat Cloud2RangeNode::SavitskyGolaySmoothing(const Mat& image, int window_size)
  {
    Mat kernel = GetSavitskyGolayKernel(window_size);

    Mat smoothed_image;  // init an empty smoothed image
    cv::filter2D(image, smoothed_image, SAME_OUTPUT_TYPE, kernel, ANCHOR_CENTER, 0, cv::BORDER_REFLECT101);

    return smoothed_image;
  }

  Mat Cloud2RangeNode::EraseGroundBFS (Mat range_image, Mat smoothed_image, double ground_angle_threshold, double start_angle_threshold, int kernel_size)
  {
    Label Labeler(smoothed_image.rows, smoothed_image.cols);
    //Mat lable_image = Labeler.GetLabelImage();
    Mat res_image = Mat::zeros(range_image.size(), CV_16UC1);
    for(int c = 0; c < n_cols_; c++)
    {
      int r = n_beams_ - 1;
      while (r > 0 && range_image.at<ushort>(r, c) < 1) {
      --r;
      }//ver se dá para tirar chavetas
      
      uint16_t current_label = Labeler.CheckLabelAt(r, c);    
      if (current_label > 0){
        // this coord was already labeled, skip
        continue;
      }

      if (smoothed_image.at<ushort>(r, c) > start_angle_threshold * 100){
        // *100 devido ao valor dentro da matriz ser um número inteiro (resolução de 0.57º)
        continue;
      }

      Labeler.LabelOneComponent(1, r, c, ground_angle_threshold, range_image, smoothed_image);
    }

    auto label_image_ptr = Labeler.GetLabelImagePtr();
    if (label_image_ptr->rows != res_image.rows || label_image_ptr->cols != res_image.cols) {
      fprintf(stderr, "ERROR: label image and res do not correspond.\n");
      return res_image;
    }

    kernel_size = std::max(kernel_size - 2, 3); //talvez seja desnecessário
    Mat kernel = GetUniformKernel(kernel_size, CV_8U);
    Mat dilated = Mat::zeros(label_image_ptr->size(), label_image_ptr->type());
    cv::dilate(*label_image_ptr, dilated, kernel);
    for (int r = 0; r < dilated.rows; ++r) {
      for (int c = 0; c < dilated.cols; ++c) {
        if (dilated.at<uint16_t>(r, c) == 0) // all unlabeled points are non-ground
        res_image.at<ushort>(r, c) = range_image.at<ushort>(r, c);
    }
  }
    return res_image;
  }


}  // namespace cloud2range

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud2range");
  cloud2range::Cloud2RangeNode node(ros::NodeHandle("~"));
  ros::spin();
}
