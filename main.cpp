 #include "ros/ros.h"
#include "alfa_gs.h"

#define NODE_NAME "alfa_gs"

#define NODE_TYPE "Ground Segmentation"


int main(int argc, char** argv)
{

     ros::init (argc, argv, "alfa_node");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }
       alfa_msg::ConfigMessage parameter1,parameter2,parameter3,parameter4,parameter5,parameter6;

       parameter1.config = 7;
       parameter1.config_name = "Filter Selector";

       parameter2.config = 0.1;
       parameter2.config_name = "Minimal Search Radius:";

       parameter3.config = 0.2;
       parameter3.config_name = "Multiplication Parameter:";

       parameter4.config = 5;
       parameter4.config_name = "Neighbor Threshold";

       parameter5.config = 0.005;
       parameter5.config_name = "Intensity Treshold Parameter:";

       parameter6.config = 4;
       parameter6.config_name = "Multithreading: Number of threads";

      vector<alfa_msg::ConfigMessage> default_configurations;
      default_configurations.push_back(parameter1);
      default_configurations.push_back(parameter2);
      default_configurations.push_back(parameter3);
      default_configurations.push_back(parameter4);
      default_configurations.push_back(parameter5);
      default_configurations.push_back(parameter6);


    Cloud2RangeNode new_node(NODE_NAME,NODE_TYPE,&default_configurations);
    while(ros::ok())
    {

    }
}
