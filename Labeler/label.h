#pragma once

#include <stdint.h>
#include <cv_bridge/cv_bridge.h>

namespace cloud2range {

using cv::Mat;


class Label
{  
private: 
    Mat label_image;
    
    struct pixel_coord {
    pixel_coord() : row(0), col(0) {}
    pixel_coord(uint16_t row_, uint16_t col_) : row(row_), col(col_) {}
    pixel_coord operator+(const pixel_coord& other) const {
    return pixel_coord(row + other.row, col + other.col);
  }

  uint16_t row;
  uint16_t col;
};

 const uint8_t STEP_ROW = 1, STEP_COL = 1;
// static constexpr int16_t NEIGH_SIZE = 2 * STEP_ROW + 2 * STEP_COL;
// std::array<PixelCoord, NEIGH_SIZE> Neighborhood;
std::array<pixel_coord, 4> Neighborhood;

public:
    Label(int n_rows, int n_cols);
    ~Label();
    Mat GetLabelImage();
    inline const Mat* GetLabelImagePtr() {return &label_image;}
    inline uint8_t CheckLabelAt(pixel_coord coord) {return label_image.at<uint16_t>(coord.row, coord.col);}
    inline uint8_t CheckLabelAt(int row, int col) {return label_image.at<uint16_t>(row, col);}
    inline void SetLabelAt(int label, pixel_coord coord){label_image.at<uint16_t>(coord.row, coord.col) = label;}
    uint16_t WrapCols(int16_t col);
    void LabelOneComponent(uint8_t label, int row, int col, float ground_angle_threshold, Mat range_image, Mat smoothed_image);
};


}

