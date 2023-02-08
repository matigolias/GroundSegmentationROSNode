#include "label.h"

#include <stdint.h>
#include <queue>

namespace cloud2range {

Label::Label(int n_rows, int n_cols)
{
 label_image = Mat::zeros(n_rows, n_cols, CV_8U); //Mat.at<uchar>(y,x)

 uint16_t counter = 0;
    for (uint16_t r = STEP_ROW; r > 0; --r) {
      Neighborhood[counter++] = pixel_coord(-r, 0);
      Neighborhood[counter++] = pixel_coord(r, 0);
    }
    for (uint16_t c = STEP_COL; c > 0; --c) {
      Neighborhood[counter++] = pixel_coord(0, -c);
      Neighborhood[counter++] = pixel_coord(0, c);
    }
}

Label::~Label()
{
    //label_image = Mat::zeros(n_beams_, n_cols_, CV_16UC1)
}

Mat Label::GetLabelImage()
{
    return label_image;
}

uint16_t Label::WrapCols(int16_t col) 
{
    // we allow our space to fold around cols
    if (col < 0) {
      //std::cout << "Entrei malouuucoooo - " << "\n";
      return col + label_image.cols;
    }
    if (col >= label_image.cols) {
      //std::cout << "Entrei malouuucoooo - " << "\n";
      return col - label_image.cols;
    }
    return col;
}

//Breadth First Search
void Label::LabelOneComponent(uint8_t label, int row, int col, float ground_angle_threshold, Mat range_image, Mat smoothed_image)
{
    pixel_coord start = pixel_coord(row ,col);
    std::queue<pixel_coord> labeling_queue;
    labeling_queue.push(start);
    // while the queue is not empty continue removing front point adding its
    // neighbors back to the queue - breadth-first-search one component
    size_t max_queue_size = 0;
    while (!labeling_queue.empty()) {
      max_queue_size = std::max(labeling_queue.size(), max_queue_size);
      // copy the current coordinate
      const pixel_coord current = labeling_queue.front();
      labeling_queue.pop();
      uint16_t current_label = CheckLabelAt(current);
      if (current_label > 0) {
        // we have already labeled this point. No need to add it.
        continue;
      }
      // set the label of this point to current label
      SetLabelAt( label, current);

      // check the depth
      auto current_depth = range_image.at<ushort>(current.row, current.col);
      if (current_depth < 0.001f) {
        // depth of this point is wrong, so don't bother adding it to queue
        continue;
      }
      for (const auto& step : Neighborhood) {
        pixel_coord neighbor = current + step;
        if (neighbor.row < 0 || neighbor.row >= label_image.rows) {
          // point doesn't fit
          continue;
        }
        // if we just went over the borders in horiz direction - wrap around
        neighbor.col = WrapCols(neighbor.col);
        uint16_t neigh_label = CheckLabelAt(neighbor);
        if (neigh_label > 0) {
          // we have already labeled this one
          // std::cout << "Já fostes" << "\n";
          continue;
        }

        double diff = fabs (smoothed_image.at<ushort>(current.row, current.col) - smoothed_image.at<ushort>(neighbor.row, neighbor.col));

        // std::cout << "Row - " << current.row << " Col - " << current.col << " Val - " <<  smoothed_image.at<ushort>(current.row, current.col) << "\n";
        // std::cout << "N-Row - " << neighbor.row << " N-Col - " << neighbor.col << " N-Val - " <<  smoothed_image.at<ushort>(neighbor.row, neighbor.col) << "\n";
        // std::cout << "DIFERENCE - " << diff << " GND_ANG_THRESHOLD * 100 - " << ground_angle_threshold * 100 << "\n\n";

        // if (smoothed_image.at<ushort>(neighbor.row, neighbor.col) > 60)
        // continue;

        if (diff < ground_angle_threshold * 100) {
          labeling_queue.push(neighbor);
        }
      }
    }
  }

}


