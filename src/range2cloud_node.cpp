#include "utils.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace cloud2range {

using PointT = pcl::PointXYZ;

class Range2CloudNode {
 public:
  explicit Range2CloudNode(const ros::NodeHandle& pnh);

  void CameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& cinfo_msg);

 private:
  ros::NodeHandle pnh_;
  ros::Publisher pub_cloud_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  //double HDL_64_vert_angle [64]{-0.428567, -0.428567, -0.411010, -0.400830, -0.390212, -0.383550, -0.375084, -0.367629, -0.356546, -0.347179, -0.335927, -0.330268, -0.321645, -0.313528, -0.303135, -0.293236, -0.284032, -0.277615, -0.268517, -0.260519, -0.249417, -0.240953, -0.229741, -0.223919, -0.214571, -0.207927, -0.197139, -0.187686, -0.177605, -0.170069, -0.162315, -0.153144, -0.149445, -0.143444, -0.138635, -0.131811, -0.126182, -0.119537, -0.114497, -0.108036, -0.102780, -0.095696, -0.089616, -0.083935, -0.079062, -0.072558, -0.066659, -0.060551, -0.054439, -0.048731, -0.043427, -0.036488, -0.031588, -0.025460, -0.019944, -0.013200, -0.007886, -0.001549, 0.003561, 0.010307, 0.015620, 0.022568, 0.027267, 0.034210};
};

Range2CloudNode::Range2CloudNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh) {
  sub_camera_ =
      it_.subscribeCamera("/cloud2range/range/image", 1, &Range2CloudNode::CameraCb, this);
  pub_cloud_ = pnh_.advertise<pcl::PointCloud<PointT>>("cloud_ordered", 1);
}

void Range2CloudNode::CameraCb(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  // Convert to image
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat range_image = cv_ptr->image;

  // Extract params from cinfo
  const double min_angle = cinfo_msg->K[0];
  //  const double max_angle = cinfo_msg->K[1];
  const double min_range = cinfo_msg->K[2];
  const double max_range = cinfo_msg->K[3];
  const double d_azimuth = cinfo_msg->K[4];
  const double d_altitude = cinfo_msg->K[5];

  int n_beams_ = cinfo_msg-> height;

  // Create a point cloud and fill in points from range image
  pcl::PointCloud<PointT> cloud;
  cloud.points.reserve(range_image.rows * range_image.cols);

  for (int r = 0; r < range_image.rows; ++r) {
    const auto row_ptr = range_image.ptr<ushort>(r);
    for (int c = 0; c < range_image.cols; ++c) {
      const ushort range_encoded = row_ptr[c];

      // skip points with 0 range
      if (range_encoded == 0) {
        continue;
      }

      const double range_norm = static_cast<double>(range_encoded - 1) /
                                (std::numeric_limits<ushort>::max() - 1);
      const double range = range_norm * (max_range - min_range) + min_range;

      //ROS_INFO("RANGE %f",range);

      //const auto altitude = r * d_altitude + min_angle;
      const auto altitude = (range_image.rows - r) * d_altitude + min_angle;
      //const auto altitude = HDL_64_vert_angle[n_beams_-1-r];
      const auto azimuth = c * d_azimuth;

      PointT point;
      point.x = std::cos(altitude) * std::cos(azimuth) * range;
      point.y = std::cos(altitude) * std::sin(azimuth) * range;
      point.z = std::sin(altitude) * range;
      cloud.points.push_back(point);
    }
  }

  ROS_DEBUG("num restored points %zu", cloud.size());
  ROS_INFO("num restored points %zu", cloud.size());

  pcl_conversions::toPCL(image_msg->header, cloud.header);
  cloud.header.frame_id = "os_sensor"; //velodyne // vlp16_port //velo_link //os_sensor
  pub_cloud_.publish(cloud);
}

}  // namespace cloud2range

int main(int argc, char** argv) {
  ros::init(argc, argv, "range2cloud");
  cloud2range::Range2CloudNode node(ros::NodeHandle("~"));
  ros::spin();
}
