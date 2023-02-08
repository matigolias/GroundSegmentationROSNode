//ROS includes
#include <ros/ros.h>

//PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

//ALFA Includes
#include "alfa_msg/AlfaConfigure.h"
#include "alfa_msg/AlfaMetrics.h"
#include "alfa_msg/AlfaAlivePing.h"
#include "utils.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "Labeler/label.h"
#define TIMER_SLEEP 5000 // Time between alive messages

#define CLOUD_TOPIC "alfa_pointcloud"  //Name of the subsriver topic where this node gets point clouds

typedef long long int u64;
using cv::Mat;

//#define DEBUG
//#define HARDWARE

using namespace std;

class AlfaNode
{
public:
    /**
     * @brief Construct a new Alfa Node object
     * 
     * @param node_name Define the name of the node. This is used to define all the associeted topic names 
     * @param node_type Defines the node type. This is used to classify this node depending on its function.
     * @param default_configurations Default configurations to serve as a base line for configurations using the ALFA-Monitor.
     */
    AlfaNode(string node_name, string node_type, vector<alfa_msg::ConfigMessage> *default_configurations);

    /**
     * @brief Function that publishes a point cloud in a ROS Topic
     * 
     * @param input_cloud Point cloud that will be published in a ROS Topic
     */
    void publish_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    /**
     * @brief Function that publishes a range image in a ROS Topic
     * 
     * @param range_img Range image that will be published in a ROS Topic
     */
    void publish_range_img(Mat range_img, sensor_msgs::CameraInfo cinfo_);

    /**
     * @brief Function that publishes a colored image in a ROS Topic
     * 
     * @param range_img Range image that will be published in a ROS Topic
     */
    void publish_colored_img(Mat range_img, sensor_msgs::CameraInfo cinfo_);

    /**
     * @brief Function that published the collected metrics during execution in a ROS Topic
     * 
     * @param metrics The metrics that will be published in a ROS Topic
     */
    void publish_metrics(alfa_msg::AlfaMetrics &metrics);

    /**
     * @brief Function that is called when a new point cloud is avaliable in the ROS Topic. This needs to be implemented in the child class
     * 
     * @param input_cloud The point cloud that is recieved from the ROS Topic
     */
    virtual void process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    /**
     * @brief Function that is called with a call to the configuration service is made
     * 
     * @param req The received configurations to be aplied
     * @return alfa_msg::AlfaConfigure::Response An int that represents the success of the configurations
     */
    virtual alfa_msg::AlfaConfigure::Response process_config(alfa_msg::AlfaConfigure::Request &req);

    /**
     * @brief store_pointcloud_hardware Stores a entire point cloud in the given memory regions. Every point in this point cloud
     * is converted to a 64bits memory possition
     *
     * @param input_cloud The point cloud that will be saved
     * @param pointer The pointer to the first position of the memory region where the point cloud will be saved
     */
    virtual void store_pointcloud_hardware(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, u64 *pointer);

    // /**
    //  * @brief read_hardware_pointcloud Reads and returns a point cloud. This point cloud is converted from 64bits points to the pcl format
    //  *
    //  * @param pointer The pointer to the first position of the memory region where the point cloud will be read
    //  * @param size The number of points to be read from the memory
    //  * @return Returns a pcl object with points in the PointXYZI format
    //  */
    // virtual pcl::PointCloud<pcl::PointXYZI>::Ptr  read_hardware_pointcloud(u64 *pointer, uint size);

    /**
     * @brief read_hardware_pointcloud Reads and returns a range image.
     *
     * @param pointer The pointer to the first position of the memory region where the point cloud will be read
     * @param rows The number of RI rows to be read from the memory
     * @param cols The number of RI cols to be read from the memory
     * @return Returns a pcl object with points in the PointXYZI format
     */
    virtual Mat read_hardware_pointcloud(u64 *pointer, uint rows, uint cols);

    /**
     * @brief read_hardware_filtered_angle_image  Reads and returns a angle image.
     *
     * @param pointer The pointer to the first position of the memory region where the point cloud will be read
     * @param rows The number of AI rows to be read from the memory
     * @param cols The number of AI cols to be read from the memory
     * @return Returns a pcl object with points in the PointXYZI format
     */ 
    virtual Mat read_hardware_filtered_angle_image(u64 *pointer, uint rows, uint cols);


    /**
     * @brief read_hardware_registers Reads a user-defined number of 32bits registers. Useful to communicate with AXI-Lite registers
     * @param pointer The pointer to the register that will be read
     * @param size The number of registers to be read by this function
     * @return A vector containing all the registers thar where read
     */
    virtual vector<uint32_t> read_hardware_registers(uint32_t* pointer, uint size);


    /**
     * @brief write_hardware_registers Writes a user-defined number of 32bits registers. Useful to communicate with AXI-Lite registers
     * @param data The data that will be stored in the 32bit registers
     * @param pointer The pointer to the register that will be stored
     * @param offset Offset from the pointer to skip some registers
     */
    virtual void  write_hardware_registers(vector<uint32_t> data, uint32_t* pointer, uint offset = 0);

    

    /**
     * @brief Variable responsible for signaling the current state of the node. This status is sent with all the message in the Alive Message that is sent periodically
     * 
     */
    int node_status;
    vector<alfa_msg::ConfigMessage> *default_configurations;
    virtual ~AlfaNode();

private:

    #ifndef HARDWARE
    /**
     * @brief Callback of the subcrived point cloud topic. Every point cloud published will trigger a call to this function where the point cloud is converted to the
     *  pcl XYZI format
     */
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud); 
    
    /**
     * @brief The point cloud subscriver
     * 
     */
    ros::Subscriber sub_cloud;
    #endif
    #ifdef HARDWARE
    /**
     * @brief cloud_hcb Callback that is triggered by an hardware interrupt, and reads a point cloud.
     */
    void cloud_hcb();

    /**
     * @brief publish_pointcloud publish the pointcloud that was gathered from the hardware layers
     * @param input_cloud
     */
    void publish_hardware_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    /**
     * @brief hardware_cloud_publisher publisher of the hardware point cloud
     */
    ros::Publisher hardware_cloud_publisher;
    #endif
    /**
     * @brief Callback of the configuration service that this node class provides. It receives a set of configurations and sends the configuration result, if it was
     * successfully or not
     * 
     * @param req The configurations recieved
     * @param res The response sent back
     * @return true 
     * @return false 
     */
    bool parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res);


    /**
     * @brief the service server of the configurations
     * 
     */
    ros::ServiceServer sub_parameters;

    /**
     * @brief A ROS Nodehandler
     * 
     */
    ros::NodeHandle nh;

    /**
     * @brief Intern point cloud that is used in conversions
     * 
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud;

    /**
     * @brief Initializes the ROS environment of this node
     * 
     */
    void init();

    /**
     * @brief Function responsible for subscribing to all the topic needed to enable full communication with ALFA-Monitor
     * 
     */
    void subscribe_topics();

    /**
     * @brief Runs on another trhead, and is responsible for publishing the alive messages
     * 
     */
    void ticker_thread();

    /**
     * @brief thread that runs the ros "spin"
     * 
     */
    boost::thread *m_spin_thread;

    /**
     * @brief publisher of the node metrics
     * 
     */
    ros::Publisher node_metrics;

    /**
     * @brief publisher of the alive messages
     * 
     */
    ros::Publisher alive_publisher;

    /**
     * @brief publisher of the point cloud
     * 
     */
    ros::Publisher cloud_publisher;
    image_transport::CameraPublisher range_img_publisher;
    image_transport::ImageTransport *it_;

    /**
     * @brief thread that runs the "ticker_thread"
     * 
     */
    boost::thread *alive_ticker;

    string node_name;
    string node_type;
    void spin();
    uint pcl2_Header_seq;
};
