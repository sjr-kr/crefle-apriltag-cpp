#ifndef CAMERA_CALIBRATION_HPP
#define CAMERA_CALIBRATION_HPP

#include <opencv2/opencv.hpp>

void capture_calibration_images(int camera_index, int num_corners_x, int num_corners_y, float square_size);

#endif // CAMERA_CALIBRATION_HPP
