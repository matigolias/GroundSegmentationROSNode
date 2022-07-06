#include "alfa_node.h"
#include <thread>
#include <unistd.h>
#include <chrono>


#define RES_MULTIPLIER 10
#define INTENSITY_MULTIPLIER 1000
//#define DEBUG

AlfaNode::AlfaNode(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations ) //: it_(nh) 
{
    this->node_name = node_name;
    this->node_type = node_type;
    this->default_configurations = default_configurations;
    pcl2_Header_seq = 0;
    pcloud.reset(new pcl::PointCloud<pcl::PointXYZI>); // Create a new point cloud object
    init(); //inicialize the ROS enviroment
    it_ = new  image_transport::ImageTransport(nh);
    subscribe_topics();  //Subscrive to all the needed topics
    alive_ticker = new boost::thread(&AlfaNode::ticker_thread,this); //Start the ticker thread that sends the alive message


}

void AlfaNode::publish_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // cloud.header.frame_id = node_name+"_pointcloud";  // Create the pointcloud2 header to publish
    // cloud.header.seq = pcl2_Header_seq;
    // //cloud.header.stamp = ros::Time::now();
    // pcl2_Header_seq++;
    // cloud_publisher.publish(cloud); //publish the point cloud in the ROS topic
    

    sensor_msgs::PointCloud2 pcl2_frame;
    pcl::toROSMsg(*cloud, pcl2_frame);   //conver the pcl object to the pointcloud2 one
    pcl2_frame.header.frame_id = node_name+"_pointcloud";  // Create the pointcloud2 header to publish
    pcl2_frame.header.seq = pcl2_Header_seq;
    pcl2_frame.header.stamp = ros::Time::now();
    pcl2_Header_seq++;
    cloud_publisher.publish(pcl2_frame); //publish the point cloud in the ROS topic
}

void AlfaNode::publish_range_img(Mat range_img, sensor_msgs::CameraInfo cinfo_) 
{

     sensor_msgs::PointCloud2 pcl2_frame;
    // pcl2_frame.header.frame_id = node_name+"_pointcloud";  // Create the pointcloud2 header to publish
    // pcl2_frame.header.seq = pcl2_Header_seq;
    // pcl2_frame.header.stamp = ros::Time::now();
    // pcl2_Header_seq++;
    cinfo_.header = pcl2_frame.header;


    cv_bridge::CvImage cv_image(pcl2_frame.header, "mono16", range_img);/////// res_image
    range_img_publisher.publish(*cv_image.toImageMsg(), cinfo_); // @todo retirar cinfo?

}

void AlfaNode::publish_metrics(alfa_msg::AlfaMetrics &metrics)
{
    node_metrics.publish(metrics);  // publish the metrics
}

void AlfaNode::process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    cout << "Please implement the process_pointcloud function"<<endl; //If this line execute, it means that the real function was not implemented. Please implement in the derived node
}

alfa_msg::AlfaConfigure::Response AlfaNode::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    cout << "Please implement the process_config function"<<endl; //If this line execute, it means that the real function was not implemented. Please implement in the derived node
}



void AlfaNode::store_pointcloud_hardware(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, u64 *pointer)
{
    int pointcloud_index = 0;
    int16_t a16_points[4];
    for (auto point :*input_cloud) {
        a16_points[0] = point.x*RES_MULTIPLIER;
        a16_points[1] = point.y*RES_MULTIPLIER;
        a16_points[2] = point.z*RES_MULTIPLIER;
        a16_points[3] = point.intensity*INTENSITY_MULTIPLIER;
        memcpy((void*)(pointer+pointcloud_index),a16_points,sizeof(int16_t)*4);
        pointcloud_index++;
    }
}


pcl::PointCloud<pcl::PointXYZI>::Ptr AlfaNode::read_hardware_pointcloud(u64 *pointer, uint size)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr return_cloud;
    return_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    for (uint i=0; i<size;i++) {
        pcl::PointXYZI p;
        int16_t a16_points[4];
        memcpy((void*)(a16_points), pointer+i,sizeof(int16_t)*4);
        p.x = (a16_points[0])/float(RES_MULTIPLIER);
        p.y = (a16_points[1])/float(RES_MULTIPLIER);
        p.z = (a16_points[2])/float(RES_MULTIPLIER);
        p.intensity = (a16_points[3])/float(INTENSITY_MULTIPLIER);
        return_cloud->push_back(p);
        #ifdef DEBUG

        cout<< "First bits: "<< hex<< a16_points[0]<< " Secound bits: "<< hex<< a16_points[1]<<endl;
        cout << "Obtained coordinate: X:"<< hex<< p.x<< "; Y: "<<hex <<p.y<< "; Z: "<<hex<<p.z<< "; Intensity: "<<p.intensity<<endl;
        #endif

    }
    return return_cloud;
}

vector<uint32_t> AlfaNode::read_hardware_registers(uint32_t *pointer, uint size)
{
    vector<uint32_t> return_vector;
    for (uint var = 0; var < size; ++var) {
        return_vector.push_back(pointer[var]);
    }
    return return_vector;
}

void AlfaNode::write_hardware_registers(vector<uint32_t> data, uint32_t *pointer, uint offset)
{
    for(uint i = offset; i <data.size(); i++)
    {
        pointer[i] = data[i];
    }
}



AlfaNode::~AlfaNode()
{

}


#ifndef HARDWARE
void AlfaNode::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    if ((cloud->width * cloud->height) == 0)
    {
        #ifdef DEBUG
            cout <<"Recieved empty point cloud"<<endl;
        #endif
        return;
    }
    /**
     * @brief pcl::fromROSMsg
     * @todo Mudar para formato "hardware"
     */
    pcl::fromROSMsg(*cloud,*pcloud); //conversion of the pointcloud2 object to the pcl one

    #ifdef DEBUG
        cout<<"Recieved a point cloud with: "<< pcloud->size()<<" points"<<endl;
    #endif
        
    process_pointcloud(pcloud);  // call the child object with the recived point cloud
    
}
#endif


#ifdef HARDWARE
void AlfaNode::cloud_hcb()
{
    /**
      @todo Exectution flow when the hardware triggers an interrupt

      */
    publish_hardware_pointcloud(pcloud);
}

void AlfaNode::publish_hardware_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    sensor_msgs::PointCloud2 pcl2_frame;
    pcl::toROSMsg(*input_cloud,pcl2_frame);   //conver the pcl object to the pointcloud2 one
    pcl2_frame.header.frame_id = node_name+"_hardware_pointcloud";  // Create the pointcloud2 header to publish
    pcl2_frame.header.seq = pcl2_Header_seq;
    pcl2_frame.header.stamp = ros::Time::now();
    pcl2_Header_seq++;
    hardware_cloud_publisher.publish(pcl2_frame);
}
#endif

bool AlfaNode::parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res)
{
    #ifdef DEBUG
        cout<<"Recieved configurations with size" <<req.configurations.size()<<"... Updating"<<endl;
        for (int i=0; i< req.configurations.size();i++) {
            cout <<"Configuration: "<<i<< " With name: "<< req.configurations[i].config_name<< " with value: "<< req.configurations[i].config<<endl;
        }
    #endif

    res = process_config(req); // process the new configurantion and prepare the result
    return true;
}

void AlfaNode::init()
{
        char arg0[]= "filter_node";
        char *argv[]={arg0,NULL};
        int argc=(int)(sizeof(argv) / sizeof(char*)) - 1;;
        ros::init (argc, argv, node_name);
          if (!ros::master::check()) {
              cout <<"Failed to inicialize ros"<<endl;
            return;
          }

}

void AlfaNode::subscribe_topics()
{

    #ifndef HARDWARE
        sub_cloud = nh.subscribe(string(CLOUD_TOPIC),1,&AlfaNode::cloud_cb,this);  //subscribe

    #endif
    sub_parameters = nh.advertiseService(string(node_name).append("_settings"),&AlfaNode::parameters_cb,this);
    ros::NodeHandle n;
    #ifdef HARDWARE
        hardware_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>(node_name.append("_hardware_cloud"),1);
    #endif
    range_img_publisher = it_->advertiseCamera("range/image", 1);
    node_metrics = n.advertise<alfa_msg::AlfaMetrics>(string(node_name).append("_metrics"), 1);
    alive_publisher = n.advertise<alfa_msg::AlfaAlivePing>(string(node_name).append("_alive"),1);
    cloud_publisher = n.advertise<sensor_msgs::PointCloud2>(string(node_name).append("_cloud"),1);
    m_spin_thread = new boost::thread(&AlfaNode::spin, this);


}

void AlfaNode::ticker_thread()
{
    while(ros::ok())
    {
        alfa_msg::AlfaAlivePing newPing;
        newPing.node_name= node_name;
        newPing.node_type = node_type;
        newPing.config_service_name = node_name+"_settings";
        newPing.config_tag = "Default configuration";
        newPing.default_configurations = *default_configurations;
        newPing.current_status = node_status;
        alive_publisher.publish(newPing);
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMER_SLEEP));
    }
}

void AlfaNode::spin()
{
    cout<<"started spinning with success"<<endl;
    ros::spin();
}

