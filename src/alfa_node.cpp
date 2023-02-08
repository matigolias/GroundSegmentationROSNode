#include "alfa_node.h"
#include <thread>
#include <unistd.h>
#include <chrono>


#define RES_MULTIPLIER 100
#define RANGE_MULTIPLIER 100
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



// void AlfaNode::store_pointcloud_hardware(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, u64 *pointer)
// {
//     int pointcloud_index = 0;
//     int16_t a16_points[4];
//     for (auto point :*input_cloud) {
//         a16_points[0] = point.x*RES_MULTIPLIER;
//         a16_points[1] = point.y*RES_MULTIPLIER;
//         a16_points[2] = point.z*RES_MULTIPLIER;
//         a16_points[3] = point.intensity*INTENSITY_MULTIPLIER;
//         memcpy((void*)(pointer+pointcloud_index),a16_points,sizeof(int16_t)*4);
//         pointcloud_index++;
//     }
// }

void AlfaNode::store_pointcloud_hardware(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, u64 *pointer)
{
    int pointcloud_index = 0;
    uint16_t a16_points[4];
    int16_t elevation;
    a16_points[3] = 0;
    for (auto point :*input_cloud){
        //elevation
        elevation = (float) ((std::atan2(point.z, std::hypot(point.x, point.y)))* (180.0f/M_PI)) *RES_MULTIPLIER;
        a16_points[0] = elevation;
        //azimuth
        const auto a = std::atan2(point.y, point.x);
        a16_points[1] = (float) ((point.y >= 0 ? a : a + M_PI * 2) * (180.0f/M_PI)) * RES_MULTIPLIER;
        //range
        a16_points[2] = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) * RANGE_MULTIPLIER;

        #ifdef DEBUG
        if(pointcloud_index < 16 || (pointcloud_index>=32 && pointcloud_index <=47) || (input_cloud->size()-pointcloud_index<10))
        {   
            std::cout << pointcloud_index << " RANGE: " << range << " | a16_PTS: " << a16_points[0] << "  " << a16_points[1] << "  " << a16_points[2] << " " << a16_points[3] << endl;
        }
        #endif

        memcpy((void*)(pointer+pointcloud_index),a16_points,sizeof(int16_t)*4);
        pointcloud_index++;
    }
}


// pcl::PointCloud<pcl::PointXYZI>::Ptr AlfaNode::read_hardware_pointcloud(u64 *pointer, uint size)
// {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr return_cloud;
//     return_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
//     for (uint i=0; i<size;i++) {
//         pcl::PointXYZI p;
//         int16_t a16_points[4];
//         memcpy((void*)(a16_points), pointer+i,sizeof(int16_t)*4);
//         p.x = (a16_points[0])/float(RES_MULTIPLIER);
//         p.y = (a16_points[1])/float(RES_MULTIPLIER);
//         p.z = (a16_points[2])/float(RES_MULTIPLIER);
//         //p.intensity = (a16_points[3])/float(INTENSITY_MULTIPLIER);
//         return_cloud->push_back(p);
//         #ifdef DEBUG

//         cout<< "First bits: "<< hex<< a16_points[0]<< " Secound bits: "<< hex<< a16_points[1]<<endl;
//         cout << "Obtained coordinate: X:"<< hex<< p.x<< "; Y: "<<hex <<p.y<< "; Z: "<<hex<<p.z<< "; Intensity: "<<p.intensity<<endl;
//         #endif

//     }
//     return return_cloud;
//}

Mat AlfaNode::read_hardware_pointcloud(u64 *pointer, uint rows, uint cols)
{
    // pcl::PointCloud<pcl::PointXYZI>::Ptr return_cloud;
    // return_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    Mat hw_RI = Mat::zeros(64, 1800, CV_16UC1);

    uint size = rows * cols;
    uint ddrSize = size/4; // since each position has 16 bits, 16*4=64 bit blocks
    uint8_t row = 0;
    uint16_t col = 0;
    // unsigned char* data = new unsigned char[size];
    // unsigned char* dataPtr = data;
    for (uint i=0; i<ddrSize; i++) {
        uint16_t a16_points[4];
        memcpy((void*)(a16_points), pointer+i, sizeof(uint16_t)*4);
        for(uint j=0; j<4; j++){
            if(row>=64)
            {
                col++;
                row=0;
                cout << "COL ->" << col << endl;
            }
                //cout << "addr ----" << pointer+i << endl;
                hw_RI.at<ushort>(row, col) = a16_points[j]/100;
                row++;
        }     
    }
    cout << "DDR Size ->" << ddrSize << endl; 
        
        #ifdef DEBUG
        // cout<< "First bits: "<< hex<< a16_points[0]<< " Second bits: "<< hex<< a16_points[1]<<endl;
        // cout << "Obtained coordinate: X:"<< hex<< p.x<< "; Y: "<<hex <<p.y<< "; Z: "<<hex<<p.z<< "; Intensity: "<<p.intensity<<endl;
        #endif

    return hw_RI;
}
    
Mat AlfaNode::read_hardware_filtered_angle_image(u64 *six_points, uint rows, uint cols)
{
    Mat hw_AI = Mat::zeros(rows, cols, CV_16UC1);

    uint size = rows * cols;
    uint ddrSize = size/4; // since each position has 16 bits, 16*4=64 bit blocks
    uint16_t row = 0;
    uint16_t col = 0;
    const uint16_t ten_bit_mask = 0x3FF;  
    uint32_t point_cntr = 0;
    uint8_t burst_cntr = 0;

    while (point_cntr < size)
    {  
        //uint16_t a16_points[4];
        //memcpy((void*)(a16_points), pointer+i, sizeof(uint16_t)*4);

        if(burst_cntr == 42)
        {
            for(int k=0; k<4; k++){
            if(row>=63)//colocar macro rows
            {
                col++;
                row=0;
            }
                hw_AI.at<ushort>(row, col) = *six_points & ten_bit_mask; 

                // cout << "----------------4 Pontos------------------" << endl;
                // cout << "addr " << six_points << endl;
                // cout << "ORIGINAL " << std::hex << *(six_points) << endl;
                // cout << "SEGMENTED " << std::hex << hw_AI.at<ushort>(row, col) << endl;
                // //cout << "angle ->" << hw_AI.at<ushort>(row, col) << endl;
                // printf("angle * 100 -> %d", hw_AI.at<ushort>(row, col));

                // cout << "row - " << row << endl;
                // cout << "col - " << col << endl;

                *six_points = *six_points >> 10;

                row++;
                point_cntr++;
            }
            burst_cntr = 0;
        }
        else
        {
            for(uint j=0; j<6; j++){
            if(row>63)
            {
                col++;
                row=0;
            }
                hw_AI.at<ushort>(row, col) = *six_points & ten_bit_mask; 

                // cout << "-----------------//------------------" << endl;
                // cout << "addr " << six_points << endl;
                // cout << "ORIGINAL " << std::hex << *(six_points) << endl;
                // cout << "SEGMENTED " << std::hex << hw_AI.at<ushort>(row, col) << endl;
                // //cout << "angle ->" << hw_AI.at<ushort>(row, col) << endl;
                // printf("angle * 100 -> %d \n", hw_AI.at<ushort>(row, col));

                // printf("row - %d \n", row);
                // printf("col - %d \n", col);

                *six_points = *six_points >> 10;

                row++;
                point_cntr++;
            } 
            burst_cntr ++;  
        }  

        six_points = six_points +1;
    }
    cout << "Angle Image" << endl; 
        
        #ifdef DEBUG
        // cout<< "First bits: "<< hex<< a16_points[0]<< " Second bits: "<< hex<< a16_points[1]<<endl;
        // cout << "Obtained coordinate: X:"<< hex<< p.x<< "; Y: "<<hex <<p.y<< "; Z: "<<hex<<p.z<< "; Intensity: "<<p.intensity<<endl;
        #endif

    return hw_AI;
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

