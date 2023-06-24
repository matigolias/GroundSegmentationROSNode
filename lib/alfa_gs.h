#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "alfa_msg/AlfaConfigure.h"
#include "alfa_msg/AlfaMetrics.h"
#include "alfa_node.h"

#include "utils.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "Labeler/label.h"

#include <math.h>

using namespace std;
using PointT = pcl::PointXYZRGB;
using cv::Mat;

const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
const int SAME_OUTPUT_TYPE = -1;

typedef long long int u64;

class Alfa_GS :  public  AlfaNode //mudar para Alfa_GS
{
public:
    Alfa_GS(string node_name, string node_type, vector<alfa_msg::ConfigMessage>* default_configurations);

    Mat CreateRangeImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    Mat RepairGaps(const Mat no_ground_image, int step, float depth_threshold);
    Mat CreateAngleImg(Mat range_image);
    Mat CreateAngleImg2(const Mat &range_image);
    Mat MovingAverageSmoothing(Mat angle_image, int window_size);
    Mat CreateResImage(Mat range_image, Mat smoothed_image);
    // int FindRow(int unprecise_row, float vert_angle);
    Mat GetUniformKernel(int window_size, int type);
    Mat GetSavitskyGolayKernel(int window_size);
    Mat SavitskyGolaySmoothing(const Mat& image, int window_size);
    Mat EraseBFS (Mat range_image, Mat smoothed_image, double ground_angle_threshold, double start_angle_threshold, int kernel_size);
    Mat CreateColoredAngleImage(Mat angle_image);
    void CheckNumberOfDetectedRIdGnd (Mat og_range_image, Mat seg_range_image, Mat labeled_range_image);
    Mat CreateLabeledRangeImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

    pcl::PointCloud<PointT>::Ptr CameraCb(cv::Mat range_image, const sensor_msgs::CameraInfo cinfo_);
    void write_hardware_configurations();

    alfa_msg::AlfaMetrics outputMetrics;

private:

  int n_beams_,rpm_, sample_freq_;
  double min_angle_, max_angle_;
  double min_range_, max_range_;

  double d_azimuth_, d_altitude_;
  double start_angle_threshold, ground_angle_threshold;
  int n_cols_, window_size;
  int erase_ground;

  int sensor_tag;
  bool hw;

  // We are going to use camera info to store all the above params for decoding
  sensor_msgs::CameraInfo cinfo_;

  u64 *ddr_pointer;
  u64 *ddr_pointer_2;
  u_int32_t *hw32_vptr;

  void process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  input_cloud);
  void UpdateSegmentationSettings(const alfa_msg::AlfaConfigure::Request configs);
  alfa_msg::AlfaConfigure::Response process_config(alfa_msg::AlfaConfigure::Request &req);

};
