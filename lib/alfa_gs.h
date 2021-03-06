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
using PointT = pcl::PointXYZI;
using cv::Mat;

const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
const int SAME_OUTPUT_TYPE = -1;

class Cloud2RangeNode :  public  AlfaNode //mudar para Alfa_GS
{
public:
    Cloud2RangeNode(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations);

    Mat RepairGaps(const Mat no_ground_image, int step, float depth_threshold);
    Mat CreateAngleImg(Mat range_image);
    Mat CreateResImage(Mat range_image, Mat smoothed_image);
    // int FindRow(int unprecise_row, float vert_angle);
    Mat GetUniformKernel(int window_size, int type);
    Mat GetSavitskyGolayKernel(int window_size);
    Mat SavitskyGolaySmoothing(const Mat& image, int window_size);
    Mat EraseGroundBFS (Mat range_image, Mat smoothed_image, double ground_angle_threshold, double start_angle_threshold, int kernel_size);
    void CheckNumberOfDetectedRIdGnd (Mat og_range_image, Mat seg_range_image, Mat labeled_range_image);

    pcl::PointCloud<PointT>::Ptr CameraCb(cv::Mat range_image, const sensor_msgs::CameraInfo cinfo_);

    alfa_msg::AlfaMetrics outputMetrics;

private:

  int n_beams_,rpm_, sample_freq_;
  double min_angle_, max_angle_;
  double min_range_, max_range_;

  double d_azimuth_, d_altitude_;
  double start_angle_threshold, ground_angle_threshold;
  int n_cols_, window_size;

  // We are going to use camera info to store all the above params for decoding
  sensor_msgs::CameraInfo cinfo_;

  void process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr  input_cloud);
  alfa_msg::AlfaConfigure::Response   process_config(alfa_msg::AlfaConfigure::Request &req);

};
