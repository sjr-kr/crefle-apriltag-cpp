#include "camera_calibration.hpp"
#include <iostream>

void capture_calibration_images(int camera_index, int num_corners_x, int num_corners_y, float square_size) {
    cv::VideoCapture cap(camera_index);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return;
    }

    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size board_size(num_corners_x, num_corners_y);
    int image_count = 0;
    cv::Size image_size; // 이미지 크기를 저장할 변수

    while (image_count < 20) { // 20개의 이미지를 캡처합니다.
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        if (image_size.empty()) {
            image_size = frame.size();
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, board_size, corners);

        cv::drawChessboardCorners(frame, board_size, corners, found);
        cv::imshow("Calibration", frame);

        int key = cv::waitKey(1);
        if (key == ' ' && found) { // 스페이스바를 눌러 캡처
            image_points.push_back(corners);
            image_count++;
            std::cout << "Captured image " << image_count << std::endl;
        }
        if (key == 27) break; // ESC를 눌러 종료
    }

    cv::destroyAllWindows();

    if (image_points.size() < 3) {
        std::cerr << "Error: Not enough points for calibration." << std::endl;
        return;
    }

    std::vector<std::vector<cv::Point3f>> object_points(1);
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            object_points[0].push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }
    object_points.resize(image_points.size(), object_points[0]);

    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    // gray.size() 대신 저장해둔 image_size를 사용합니다.
    cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs);

    // 캘리브레이션 데이터를 프로젝트 루트에 저장하도록 경로 수정
    cv::FileStorage fs("../calibration_data.yml", cv::FileStorage::WRITE);
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
    fs.release();

    std::cout << "Calibration successful. Data saved to ../calibration_data.yml" << std::endl;
}

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
