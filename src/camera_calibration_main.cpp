#include "camera_calibration.hpp"
#include <iostream>

int main() {
    int camera_index, num_corners_x, num_corners_y;
    float square_size;

    std::cout << "Enter camera index: ";
    std::cin >> camera_index;
    std::cout << "Enter number of inner corners on checkerboard (X Y): ";
    std::cin >> num_corners_x >> num_corners_y;
    std::cout << "Enter square size (in meters): ";
    std::cin >> square_size;

    capture_calibration_images(camera_index, num_corners_x, num_corners_y, square_size);

    return 0;
}
