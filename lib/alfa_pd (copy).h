#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "alfa_msg/AlfaConfigure.h"
#include "alfa_msg/AlfaMetrics.h"
#include "alfa_node.h"


#define DDR_SIZE 0x060000;
#define CONFIG_SIZE 0x0006;
#define DDR_BASE_PTR 0x0F000000;
#define CONFIGS_BASE_PTR 0xA0000000;

using namespace std;
typedef long long int u64;

class Alfa_Pd :  public  AlfaNode
{
public:
    Alfa_Pd(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations);
    void do_voxelfilter();
    void do_sorfilter();
    void do_drorfilter();
    void do_rorfilter();
    void do_fcsorfilter();
    void do_LIORfilter();
    void do_DIORfilter();
    void do_hardwarefilter();

    pcl::PointCloud<pcl::PointXYZI>::Ptr apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud);
    void update_filterSettings(const alfa_msg::AlfaConfigure::Request configs);

    alfa_msg::AlfaMetrics outputMetrics;

private:
    unsigned int filter_number;
    float parameter1;
    float parameter2;
    float parameter3;
    float parameter4;
    float parameter5;
    float parameter6;
    bool use_multi;
    bool hardware_ready;
    int number_threads;
    long unsigned int frame_id;
    vector<boost::thread *> thread_list;
    u64 *ddr_pointer;
    uint32_t *configs_pointer;

    void run_worker(int thread_number);
    void run_lior_worker(int thread_number);
    void run_dior_worker(int thread_number);
    bool filter_point(pcl::PointXYZI point,pcl::KdTreeFLANN<pcl::PointXYZI> kdtree);
    bool filter_pointROR(pcl::PointXYZI point,pcl::KdTreeFLANN<pcl::PointXYZI> kdtree);
    void decode_pointcloud();
    boost::mutex mutex;


    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud, outputCloud;


    void process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr  input_cloud);
     alfa_msg::AlfaConfigure::Response   process_config(alfa_msg::AlfaConfigure::Request &req);

};
