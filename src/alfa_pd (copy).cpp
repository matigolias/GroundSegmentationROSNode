#include "alfa_pd.h"
#include <chrono>
#include<time.h>
#include <pcl/common/io.h>

Alfa_Pd::Alfa_Pd(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations):AlfaNode (node_name,node_type,default_configurations)
{
    frame_id =0;
    filter_number = 1;
    parameter1 = 0;
    parameter2 = 0.1;
    parameter3 = 0.1;
    parameter4 = 0.1;
    parameter5 = 0.1;
    hardware_ready = 0;
    int fd;
    //int ddr_size = DDR_SIZE;
    int ddr_base_ptr = DDR_BASE_PTR;
    int config_size = CONFIG_SIZE;
    int config_base_ptr = CONFIGS_BASE_PTR;
    //Hardware parameters:
    unsigned int ddr_size = 0x060000;
    unsigned int configs_size = 0x0006;
    off_t ddr_ptr_base = 0x0F000000; // physical base address
    off_t configs_ptr_base =0xA0000000;  // physical base address
    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
         //ddr_pointer = (u64 *)mmap(NULL, ddr_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ddr_base_ptr);
         //configs_pointer = (uint32_t *)mmap(NULL, config_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, config_base_ptr);
    ddr_pointer = (u64 *)mmap(NULL, ddr_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ddr_ptr_base);
    configs_pointer = (uint32_t *)mmap(NULL, configs_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, configs_ptr_base);
    hardware_ready=1;
    }

    outputMetrics.message_tag = "Filter performance";
    inputCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    outputCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);


}

void Alfa_Pd::do_voxelfilter()
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud (inputCloud);
    vg.setLeafSize (parameter1,parameter2,parameter3);
    vg.filter (*outputCloud);
}


void Alfa_Pd::do_sorfilter()
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
       sor.setInputCloud (inputCloud);
       sor.setMeanK (parameter1);
       sor.setStddevMulThresh (parameter2);
       sor.filter (*outputCloud);
}

void Alfa_Pd::do_drorfilter()
{
   outputCloud->clear();
   kdtree.setInputCloud(inputCloud);
   number_threads = parameter5;
   if(thread_list.size()>1)
   {
       for (int i =0;i < thread_list.size();i++)
       {
           thread_list[i]->join();
        }

       thread_list.clear();
   }
   if(use_multi == true)
   {
       thread_list.clear();
       for (int i =0;i < number_threads;i++)
       {
           thread_list.push_back(new boost::thread(&Alfa_Pd::run_worker, this,i));
       }
       for (int i =0;i < number_threads;i++)
       {
           thread_list[i]->join();
       }

   }
   else {
       for (auto &point : *inputCloud)
       {
           if(filter_point(point,kdtree))
               outputCloud->push_back(point);
       }
   }

}

void Alfa_Pd::do_rorfilter()
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
    ror.setInputCloud(inputCloud);
    ror.setRadiusSearch(parameter1);
    ror.setMinNeighborsInRadius (parameter2);
    ror.filter (*outputCloud);
}

void Alfa_Pd::do_fcsorfilter()
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);
    double NumberClusters = parameter1;
    double  ClusterLength = abs(maxPt.x -minPt.x)/NumberClusters;
    double ClusterWidth = abs(maxPt.y - minPt.y)/NumberClusters;
    double ClusterHeight = abs(maxPt.z - minPt.z)/NumberClusters;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud (inputCloud);
    voxel.setLeafSize (ClusterLength,ClusterWidth, ClusterHeight);
    pcl::PointCloud<pcl::PointXYZI>::Ptr betweenCloud( new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter (*betweenCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (betweenCloud);
    sor.setMeanK (parameter2);
    sor.setStddevMulThresh (parameter3);
    sor.filter (*outputCloud);
}

void Alfa_Pd::do_LIORfilter()
{

    kdtree.setInputCloud(inputCloud);
    outputCloud->clear();
    number_threads = parameter4;

    if(thread_list.size()>1)
    {
        for (int i =0;i < thread_list.size();i++)
        {
            thread_list[i]->join();
         }

        thread_list.clear();
    }

    if (number_threads >1)
    {
        thread_list.clear();

        for (int i =0;i < number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Alfa_Pd::run_lior_worker, this,i));
        }
        for (int i =0;i < number_threads;i++)
        {
            thread_list[i]->join();
         }
    }
    else {
        for (auto &point : *inputCloud)
        {
            double intensity_trehshold=parameter2;
            if(point._PointXYZI::intensity > intensity_trehshold)
            {
                outputCloud->push_back(point);
            }
            else
            {
                if(filter_pointROR(point,kdtree))
                    outputCloud->push_back(point);

            }
        }

    }
}




void Alfa_Pd::do_DIORfilter()
{
    mutex.lock();
    outputCloud->clear();
    outputCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    kdtree.setInputCloud(inputCloud);
    mutex.unlock();
    number_threads = parameter5;
       if(thread_list.size()>1)
       {
           for (int i =0;i < thread_list.size();i++)
           {
               thread_list[i]->join();
            }

           thread_list.clear();
       }

       if (number_threads >1)
       {
           thread_list.clear();
           for (int i =0;i < number_threads;i++)
           {
               thread_list.push_back(new boost::thread(&Alfa_Pd::run_dior_worker, this,i));
           }
           for (int i =0;i < number_threads;i++)
           {
               thread_list[i]->join();
            }
       }
       else {
           for (auto &point : *inputCloud)
           {
               double intensity_trehshold=parameter4;
               if(point._PointXYZI::intensity > intensity_trehshold)
               {
                   outputCloud->push_back(point);
               }
               else
               {
                   if(filter_point(point,kdtree))
                       outputCloud->push_back(point);
               }
           }

       }

}
using namespace std::chrono;

void Alfa_Pd::do_hardwarefilter()
{
    frame_id++;
    int intensity_mult;
    vector<uint32_t> configs;
    alfa_msg::MetricMessage newMessage;
    if(parameter1 !=1)intensity_mult = parameter6;
    uint32_t config = 2+ ((((uint)parameter1)<<2)+((uint)parameter5<<6) +((uint)parameter4<<10)+((uint)parameter2<<14)+((uint)parameter3<<23));

    auto start = high_resolution_clock::now();

    store_pointcloud_hardware(inputCloud,ddr_pointer);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    newMessage.metric = duration.count();
    newMessage.units = "ms";
    newMessage.metric_name = "Storing points";
    outputMetrics.metrics.push_back(newMessage);
    configs_pointer[0]=0;
    configs.push_back(0);
    configs.push_back(0);
    configs.push_back(inputCloud->size());
    configs.push_back(frame_id-1);
    configs.push_back(frame_id);

    write_hardware_registers(configs,configs_pointer);
    configs[0] = config;
    write_hardware_registers(configs,configs_pointer);
    start = high_resolution_clock::now();

    int hardware_finish =1;
    int value = 0;
    int hardware_frame_id=0;
    usleep(10);

    while (hardware_finish) {
           vector<uint32_t> hardware_result= read_hardware_registers(configs_pointer,6);
           value = hardware_result[1];
            if(value >=1)
            {
                bool frame_differ =1;
                while (frame_differ) {
                    hardware_frame_id = hardware_result[5];
                    if(hardware_frame_id == frame_id)
                    {
                        frame_differ =0;
                    }
                }
                hardware_finish=0;
            }
            else
                 usleep(1);
    }

    configs_pointer[0] = 0;
    configs_pointer[1] = 0;
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    newMessage.metric = duration.count();
    newMessage.units = "ms";
    newMessage.metric_name = "Processing time";
    outputMetrics.metrics.push_back(newMessage);


    start = high_resolution_clock::now();
    outputCloud = read_hardware_pointcloud(ddr_pointer,inputCloud->size());
    stop = high_resolution_clock::now();

    duration = duration_cast<milliseconds>(stop - start);
    newMessage.metric = duration.count();
    newMessage.units = "ms";
    newMessage.metric_name = "Reading and decoding point cloud";
    outputMetrics.metrics.push_back(newMessage);

}


void Alfa_Pd::decode_pointcloud()
{
    outputCloud->clear();

      for (int i = 0; i < inputCloud->size(); ++i) {
          if(ddr_pointer[i]!=0)
          {
              outputCloud->push_back((*inputCloud)[i]);
          }
      }
}

void Alfa_Pd::process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{

    publish_pointcloud(apply_filter(input_cloud));
    publish_metrics(outputMetrics);
}



alfa_msg::AlfaConfigure::Response Alfa_Pd::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    update_filterSettings(req);
    alfa_msg::AlfaConfigure::Response response;
    response.return_status = 1;
    return response ;
}




pcl::PointCloud<pcl::PointXYZI>::Ptr Alfa_Pd::apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    auto start = high_resolution_clock::now();
    this->inputCloud = inputCloud;
   outputMetrics.metrics.clear();
    switch (filter_number) {
       case 1:
           do_voxelfilter();
           break;
       case 2:
           do_rorfilter();
           break;
       case 3:
           do_sorfilter();
           break;
       case 4:
           do_drorfilter();
           break;
       case 5:
           do_fcsorfilter();
           break;
       case 6:
           do_LIORfilter();
           break;
       case 7:
           do_DIORfilter();
           break;
       case 8:
           do_hardwarefilter();
           break;
    }
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<milliseconds>(stop - start);
   alfa_msg::MetricMessage newMessage;
   newMessage.metric = duration.count();
   newMessage.units = "ms";
   newMessage.metric_name = "Total processing time";
   outputMetrics.metrics.push_back(newMessage);
   newMessage.metric = inputCloud->size()-outputCloud->size();
   newMessage.units = "points";
   newMessage.metric_name = "Total removed points";
   outputMetrics.metrics.push_back(newMessage);
   return outputCloud;

}

void Alfa_Pd::update_filterSettings(const alfa_msg::AlfaConfigure::Request configs)
{
    cout<<"updating filter Settings to "<<configs.configurations[0].config<<endl;
    if(configs.configurations.size()>0)
    {
        filter_number = configs.configurations[0].config;

        switch(configs.configurations.size())
        {         
            case 7:
                parameter6 = configs.configurations[6].config;
            case 6:
                parameter5 = configs.configurations[5].config;
            case 5:
                parameter4 = configs.configurations[4].config;
            case 4:
                parameter3 = configs.configurations[3].config;
            case 3:
                parameter2 = configs.configurations[2].config;
            case 2:
                parameter1 = configs.configurations[1].config;
            break;

        }

    }
    // TODO: Control of configurations
}

bool Alfa_Pd::filter_pointROR(pcl::PointXYZI point,pcl::KdTreeFLANN<pcl::PointXYZI> kdtree)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int neighbors = kdtree.radiusSearch(point,parameter1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (neighbors >parameter3)
    {
        return true;
    }

    return false;
}



bool Alfa_Pd::filter_point(pcl::PointXYZI point,pcl::KdTreeFLANN<pcl::PointXYZI> kdtree)
{

    float distance = sqrt(pow(point.x,2)+pow(point.y,2));
    float search_radius;
    float angle;

     angle = parameter5;

    if(distance<parameter1)
    {
        search_radius = parameter1;
    }else
    {
        search_radius = parameter2 * (distance*angle);
    }
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors = kdtree.radiusSearch (point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) ;
    if(neighbors>=parameter3)
    {
        return true;
    }
    return false;
}

void Alfa_Pd::run_worker(int thread_number)
{

    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        if(filter_point(point,kdtree))
        {
            mutex.lock();
            outputCloud->push_back(point);
            mutex.unlock();
        }

    }
}

void Alfa_Pd::run_lior_worker(int thread_number)
{
    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        double intensity_trehshold = parameter2;
        float distance = sqrt(pow(point.x, 2) + pow(point.y, 2)+pow(point.z, 2));
        if(point._PointXYZI::intensity > intensity_trehshold)
        {
            mutex.lock();

            outputCloud->push_back(point);
            mutex.unlock();

        }
        else
        {
            if(filter_pointROR(point,kdtree))
            {
                mutex.lock();
                outputCloud->push_back(point);
                mutex.unlock();


             }
        }
    }
}

void Alfa_Pd::run_dior_worker(int thread_number)
{
    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        double intensity_trehshold=parameter4;
        if(point._PointXYZI::intensity > intensity_trehshold)
        {
            mutex.lock();
            outputCloud->push_back(point);
            mutex.unlock();

        }
        else
        {
            if(filter_point(point,kdtree))
            {
                mutex.lock();
                outputCloud->push_back(point);
                mutex.unlock();

            }
        }
    }

}






