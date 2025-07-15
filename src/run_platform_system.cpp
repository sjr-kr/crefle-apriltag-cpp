#include "run_platform_system.hpp"
#include "platform_logic.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

void run_platform_system() {
    cv::Mat camera_matrix, dist_coeffs;
    cv::FileStorage fs("../calibration_data.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open calibration file. Make sure you have run the calibration first." << std::endl;
        return;
    }
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs.release();

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    int camera_index;
    std::cout << "Enter camera index: ";
    std::cin >> camera_index;

    cv::VideoCapture cap(camera_index);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera with index " << camera_index << std::endl;
        return;
    }

    double marker_size, platform_side_length;
    std::cout << "Enter marker size (in meters): ";
    std::cin >> marker_size;
    std::cout << "Enter platform side length (in meters): ";
    std::cin >> platform_side_length;

    double marker_half_size = marker_size / 2.0;
    std::vector<cv::Point3f> marker_object_points = {
        cv::Point3f(-marker_half_size, -marker_half_size, 0),
        cv::Point3f( marker_half_size, -marker_half_size, 0),
        cv::Point3f( marker_half_size,  marker_half_size, 0),
        cv::Point3f(-marker_half_size,  marker_half_size, 0)
    };

    std::vector<int> platform_ids = {0, 1, 2, 3};
    int object_id = 10;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };
        zarray_t *detections = apriltag_detector_detect(td, &im);

        std::map<int, Pose> detected_markers;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            std::vector<cv::Point2f> image_points;
            image_points.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
            image_points.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
            image_points.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
            image_points.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));

            Pose p;
            cv::solvePnP(marker_object_points, image_points, camera_matrix, dist_coeffs, p.rvec, p.tvec, false, cv::SOLVEPNP_IPPE);
            
            p.center = cv::Point2d(det->c[0], det->c[1]);
            detected_markers[det->id] = p;

            cv::putText(frame, std::to_string(det->id),
                        cv::Point(det->c[0] - 10, det->c[1] + 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        }

        cv::Mat platform_rvec, platform_tvec;
        bool platform_detected = getPlatformTransform(detected_markers, platform_ids, platform_side_length, platform_rvec, platform_tvec, camera_matrix, dist_coeffs);

        if (platform_detected) {
            drawAxes(frame, camera_matrix, dist_coeffs, platform_rvec, platform_tvec, 0.1);
        }

        if (detected_markers.count(object_id)) {
            drawCube(frame, camera_matrix, dist_coeffs, detected_markers.at(object_id).rvec, detected_markers.at(object_id).tvec, marker_size);

            if (platform_detected) {
                cv::Point3f relative_pos = getRelativePosition(detected_markers.at(object_id), platform_rvec, platform_tvec);
                cv::putText(frame, cv::format("Object Pos (mm): [%.3f, %.3f, %.3f]", relative_pos.x * 1000, relative_pos.y * 1000, relative_pos.z * 1000),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                // 객체 마커의 회전 정보를 표시합니다.
                cv::Point3f euler_angles = rvecToEulerAngles(detected_markers.at(object_id).rvec);
                cv::putText(frame, cv::format("Object Rot (deg): R:%.1f P:%.1f Y:%.1f", euler_angles.x, euler_angles.y, euler_angles.z),
                            cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }
        }

        cv::imshow("Platform System", frame);

        apriltag_detections_destroy(detections);

        if (cv::waitKey(1) == 27) break; // ESC to exit
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

int main() {
    run_platform_system();
    return 0;
}
