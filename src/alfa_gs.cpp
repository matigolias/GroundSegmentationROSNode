#include "alfa_gs.h"
#include <chrono>
#include <time.h>
#include <pcl/common/io.h>
#include "lib/utils.h"

using namespace cloud2range;
using PointT = pcl::PointXYZI;
using namespace std::chrono;

Cloud2RangeNode::Cloud2RangeNode(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations):AlfaNode (node_name,node_type,default_configurations)
{
      // Read params

  window_size =  5; // 5, 7, 9, 11
  ROS_ASSERT(window_size == 5 || 7 || 9 || 11);
  ground_angle_threshold = 0.5223;//0.087266); //5º
  ROS_ASSERT(ground_angle_threshold > 0);
  start_angle_threshold = 0.523598; //30º
  ROS_ASSERT(ground_angle_threshold > 0);

  n_beams_ =  64;
  ROS_ASSERT(n_beams_ > 0);
  rpm_ = 600;
  ROS_ASSERT(rpm_ > 0);
  sample_freq_ = 18000; //512  1024  2048
  ROS_ASSERT(sample_freq_ > 0);

  min_angle_ = -0.453785;//-0.7853981634);//
  max_angle_ = 0.104719;//0.7853981634);//
  ROS_ASSERT(min_angle_ < max_angle_);

  min_range_ = 0.5;
  max_range_ = 120;
  ROS_ASSERT(min_range_ < max_range_ && min_range_ >= 0.0);

  // const auto model = pnh_.param<std::string>("model", "OS1-64");
  // ROS_INFO("lidar model: %s", model.c_str());

  // ROS_INFO(
  //     "n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], range: [%0.2f, %0.2f]",
  //     n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_), min_range_,
  //     max_range_);

  n_cols_ = sample_freq_ * 60 / rpm_;
  ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

  d_azimuth_ = Rad_Deg(360.0 / n_cols_);
  d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
  ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
           Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

  // Fill in cinfo
  cinfo_.height = n_beams_;
  cinfo_.width = n_cols_;
  //cinfo_.distortion_model = model;
  cinfo_.K[0] = min_angle_;
  cinfo_.K[1] = max_angle_;
  cinfo_.K[2] = min_range_;
  cinfo_.K[3] = max_range_;
  cinfo_.K[4] = d_azimuth_;
  cinfo_.K[5] = d_altitude_;

  sensor_tag = 64;

  unsigned int region_size = 0x10000;
  off_t axi_pbase = 0xA0000000;
  int fd;

  //unsigned int ddr_size = 0x200000;
  unsigned int ddr_size_1 = 0x90000;
  unsigned int ddr_size_2 = 0x90000;
  off_t ddr_ptr_base = 0x0F000000; // physical base address
  off_t ddr_ai_ptr_base = 0x0F100000;
  //Map the physical address into user space getting a virtual address for it
  hw = 0;

  if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
    ddr_pointer = (u64 *)mmap(NULL, ddr_size_1, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ddr_ptr_base);
    ddr_pointer_2 = (u64 *)mmap(NULL, ddr_size_2, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ddr_ai_ptr_base);
    hw32_vptr = (u_int32_t *)mmap(NULL, region_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, axi_pbase);
    hw=1;
  }
  else
  ROS_INFO("NAO ENTROU NO NMAP :(");
  
  if(hw)
    {
      write_hardware_configurations();
    }

}

void Cloud2RangeNode::process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
  if(hw)
  {
    //store point cloud
    auto start_store_hw = std::chrono::high_resolution_clock::now();
    store_pointcloud_hardware(input_cloud, ddr_pointer);
    auto stop_store_hw = std::chrono::high_resolution_clock::now();
    usleep(10);       //VERIFICAR!!!

    //create range image
    auto start_RI_hw = std::chrono::high_resolution_clock::now();
    vector<uint32_t> configs;
    configs.push_back(1);
    configs.push_back(input_cloud->size());
    write_hardware_registers(configs, hw32_vptr);
    int hw_ri_finish = 0;
    int hw_ai_finish = 0;
    //int value = 0;
    while(!hw_ri_finish){
      vector<uint32_t> hardware_result = read_hardware_registers(hw32_vptr, 4);
      //value = hardware_result[2];
      //if(value==1)
      if(hardware_result[2]==1)
        hw_ri_finish = 1;
      else
        usleep(1);
    }

    Mat hw_range_image = read_hardware_pointcloud(ddr_pointer, n_beams_, n_cols_);//mudar para read hw_RI

    while(!hw_ai_finish){
      vector<uint32_t> hardware_result = read_hardware_registers(hw32_vptr, 4);
      //value = hardware_result[2];
      //if(value==1)
      if(hardware_result[3]==1)
      {
        hw_ai_finish = 1;
        ROS_INFO("ENTROU &&&&&&&&&&&&&&&&");
      }
      else
        usleep(1);
    }

    Mat hw_angle_image = read_hardware_filtered_angle_image(ddr_pointer_2, n_beams_, n_cols_);//mudar para read hw_AI

    auto stop_RI_hw = std::chrono::high_resolution_clock::now();
    auto duration_hw_RI = duration_cast<milliseconds>(stop_RI_hw - start_RI_hw);
    ROS_INFO("TOTAL DURATION -> %ld ms", duration_hw_RI.count());

    pcl::PointCloud<PointT>::Ptr seg_point_cloud = CameraCb(hw_range_image, cinfo_);
    // update header
    //publish_range_img(hw_range_image, cinfo_); 
    publish_range_img(hw_angle_image, cinfo_);
    publish_pointcloud(seg_point_cloud);
  }
  else
  {
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

    for (size_t i = 0; i < input_cloud->size(); ++i) { //colocar em função

      //int ring=100;

      // calculate altitude and azimuth and put into range image
      const auto point = (*input_cloud)[i];
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

      int row = ((max_angle_ - altitude) / (max_angle_ - min_angle_)) * n_beams_; //atualizar com o do cloud2range

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
    ROS_DEBUG("num points %zu, num pixels %d", input_cloud->size(),
              cv::countNonZero(range_image));
    ROS_INFO("num points %zu, num pixels %d", input_cloud->size(),
              cv::countNonZero(range_image));


    ROS_INFO("INV_ALTITUDE -> %d", a);
    ROS_INFO("INV_RANGE -> %d", r);

    auto start_gs = high_resolution_clock::now();

    Mat repaired_range_image = RepairGaps(range_image, 5, 1.0f); //coolocar estas variáveis no destrutor para não haver memory leakage
    Mat angle_image = CreateAngleImg(repaired_range_image);
    Mat smoothed_image = SavitskyGolaySmoothing(angle_image, window_size);
    //Mat res_image = CreateResImage (range_image, smoothed_image);

    Mat no_ground_image = EraseGroundBFS (range_image, smoothed_image, ground_angle_threshold, start_angle_threshold, window_size);

    auto stop_gs = high_resolution_clock::now();
    auto duration_gs = duration_cast<milliseconds>(stop_gs - start_gs);

    ROS_INFO("TOTAL DURATION -> %ld ms", duration_gs.count());

    pcl::PointCloud<PointT>::Ptr seg_point_cloud = CameraCb(no_ground_image, cinfo_);

    // update header
    publish_range_img(no_ground_image, cinfo_); 
    publish_pointcloud(seg_point_cloud);
  }
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

   void Cloud2RangeNode::CheckNumberOfDetectedRIdGnd (Mat og_range_image, Mat seg_range_image, Mat labeled_range_image){
    
    int seg_gnd_cntr=0, true_detected_gnd_cntr=0, labeled_gnd_cntr=0, percentage_of_correct_gnd=0, percentage_of_incorrect_gnd=0;

    for (int col=0; col<seg_range_image.cols; ++col)
  {
    for (int row=0; row<seg_range_image.rows; ++row)
    {
      if(labeled_range_image.at<ushort>(row, col) == 0)
      labeled_gnd_cntr++;//number of true gnd points present in the range image
      if(seg_range_image.at<ushort>(row, col) == 0 && og_range_image.at<ushort>(row, col) != 0)
      seg_gnd_cntr++;//number of gnd points detected in the range image
      if(seg_range_image.at<ushort>(row, col) == labeled_range_image.at<ushort>(row, col))
      true_detected_gnd_cntr++;//number of true gnd points detected in the range image 
    }
  }

  percentage_of_correct_gnd =  (true_detected_gnd_cntr * 100 / labeled_gnd_cntr); //percentage of correct ground points detected
  percentage_of_incorrect_gnd = ((seg_gnd_cntr-true_detected_gnd_cntr) * 100 / seg_gnd_cntr); //percentage of incorrect ground points detected

  ROS_INFO("number of true gnd points present in the range image -> %d\n"
            "number of gnd points detected in the range image -> %d\n"
            "number of true gnd points detected in the range image -> %d\n"
            "percentage of correct ground points detected -> %d\n"
            "percentage of incorrect ground points detected -> %d\n", 
            labeled_gnd_cntr, seg_gnd_cntr, true_detected_gnd_cntr, percentage_of_correct_gnd, percentage_of_incorrect_gnd);

  }

  pcl::PointCloud<PointT>::Ptr Cloud2RangeNode::CameraCb(cv::Mat range_image, const sensor_msgs::CameraInfo cinfo_) {
  // Convert to image
  // cv_bridge::CvImageConstPtr cv_ptr;
  // try {
  //   cv_ptr = cv_bridge::toCvShare(image_msg);
  // } catch (cv_bridge::Exception& e) {
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }

  // const cv::Mat range_image = cv_ptr->image;

  // Extract params from cinfo
  const double min_angle = cinfo_.K[0];
  //  const double max_angle = cinfo_msg->K[1];
  const double min_range = cinfo_.K[2];
  const double max_range = cinfo_.K[3];
  const double d_azimuth = cinfo_.K[4];
  const double d_altitude = cinfo_.K[5];

  int n_beams_ = cinfo_.height;

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

  // pcl_conversions::toPCL(image_msg->header, cloud.header);
  // cloud.header.frame_id = "os_sensor"; //velodyne // vlp16_port //velo_link //os_sensor
  // pub_cloud_.publish(cloud);

  pcl::PointCloud<PointT>::Ptr ptr_cloud(new pcl::PointCloud<PointT>);

  ptr_cloud = cloud.makeShared();

  return ptr_cloud;
}

void Cloud2RangeNode::write_hardware_configurations()
{
    vector<uint32_t> configs;
    configs.push_back(0);
    configs.push_back(0);
    configs.push_back(0);
    switch(sensor_tag)
    {
      case 16:
        configs.push_back(0);                                         //d_azimuth 0.2 = azimuth LUT[0]
        configs.push_back(0);                                         //d_elevation 2 = elevation LUT[0]
        configs.push_back(0);                                         //n_lines 16 = n_lines LUT[0]
        configs.push_back(0);                                         //min_vert_angle -15 = min_vert_angle LUT[0]
        configs.push_back(0);                                         //n_columns 1800 = cols LUT[0]
        break;

      case 32:
        configs.push_back(0);                                         //d_azimuth 0.2 = azimuth LUT[0]
        configs.push_back(2);                                         //d_elevation 1.33 = elevation LUT[2]
        configs.push_back(1);                                         //n_lines 32 = n_lines LUT[1]
        configs.push_back(1);                                         //min_vert_angle -30 = min_vert_angle LUT[1]
        configs.push_back(0);                                         //n_columns 1800 = cols LUT[0]
        break;

      case 64:
        configs.push_back(0);                                         //d_azimuth 0.2 = azimuth LUT[0]
        configs.push_back(4);                                         //d_elevation 0.47 = elevation LUT[4]
        configs.push_back(2);                                         //n_lines 64 = n_lines LUT[2]
        configs.push_back(2);                                         //min_vert_angle -24.8 = min_vert_angle LUT[2]
        configs.push_back(0);                                         //n_columns 1800 = cols LUT[0]
        break;

      default:
        cout << "Invalid Sensor!" << endl;
    }
    write_hardware_registers(configs, hw32_vptr, 3);
}

alfa_msg::AlfaConfigure::Response Cloud2RangeNode::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    alfa_msg::AlfaConfigure::Response response;
    response.return_status = 1;
    return response;
}







