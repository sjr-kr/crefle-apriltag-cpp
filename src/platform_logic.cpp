#include "platform_logic.hpp"
#include <opencv2/calib3d.hpp>
#include <vector>

// 플랫폼 마커들의 3D 월드 좌표를 정의합니다.
std::map<int, cv::Point3f> getPlatformWorldCoordinates(double side_length) {
    double half_size = side_length / 2.0;
    std::map<int, cv::Point3f> world_coords;
    world_coords[0] = cv::Point3f(-half_size, -half_size, 0);
    world_coords[1] = cv::Point3f( half_size, -half_size, 0);
    world_coords[2] = cv::Point3f( half_size,  half_size, 0);
    world_coords[3] = cv::Point3f(-half_size,  half_size, 0);
    return world_coords;
}

bool getPlatformTransform(
    const std::map<int, Pose>& detected_markers,
    const std::vector<int>& platform_ids,
    double platform_side_length,
    cv::Mat& platform_rvec,
    cv::Mat& platform_tvec,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs
) {
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;

    auto platform_world_coords = getPlatformWorldCoordinates(platform_side_length);

    for (int id : platform_ids) {
        auto it = detected_markers.find(id);
        if (it != detected_markers.end()) {
            object_points.push_back(platform_world_coords.at(id));
            image_points.push_back(it->second.center);
        }
    }

    if (object_points.size() != 4) {
        return false;
    }

    cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, platform_rvec, platform_tvec, false, cv::SOLVEPNP_IPPE);

    cv::Mat R_platform;
    cv::Rodrigues(platform_rvec, R_platform);
    cv::Mat R_flip = cv::Mat::eye(3, 3, CV_64F);
    R_flip.at<double>(1, 1) = -1.0;
    R_flip.at<double>(2, 2) = -1.0;
    R_platform = R_platform * R_flip;
    cv::Rodrigues(R_platform, platform_rvec);

    return true;
}

cv::Point3f getRelativePosition(
    const Pose& object_pose,
    const cv::Mat& platform_rvec,
    const cv::Mat& platform_tvec
) {
    cv::Mat R_platform;
    cv::Rodrigues(platform_rvec, R_platform);
    cv::Mat R_platform_inv = R_platform.t();

    cv::Mat t_platform_world = platform_tvec;
    cv::Mat t_object_world = object_pose.tvec;

    cv::Mat relative_pos_mat = R_platform_inv * (t_object_world - t_platform_world);

    return cv::Point3f(
        relative_pos_mat.at<double>(0, 0),
        relative_pos_mat.at<double>(1, 0),
        relative_pos_mat.at<double>(2, 0)
    );
}

void drawAxes(cv::InputOutputArray image, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
              const cv::Mat& rvec, const cv::Mat& tvec, float length) {
    std::vector<cv::Point3f> axis_points = {
        cv::Point3f(0, 0, 0),
        cv::Point3f(length, 0, 0),
        cv::Point3f(0, length, 0),
        cv::Point3f(0, 0, length)
    };

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    cv::line(image, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 3); // X: Red
    cv::line(image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 3); // Y: Green
    cv::line(image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 3); // Z: Blue
}

void drawCube(cv::InputOutputArray image, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
              const cv::Mat& rvec, const cv::Mat& tvec, float marker_size) {
    float half_size = marker_size / 2.0f;

    std::vector<cv::Point3f> cube_points = {
        cv::Point3f(-half_size, -half_size, 0),
        cv::Point3f( half_size, -half_size, 0),
        cv::Point3f( half_size,  half_size, 0),
        cv::Point3f(-half_size,  half_size, 0),

        cv::Point3f(-half_size, -half_size, marker_size),
        cv::Point3f( half_size, -half_size, marker_size),
        cv::Point3f( half_size,  half_size, marker_size),
        cv::Point3f(-half_size,  half_size, marker_size)
    };

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(cube_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    cv::Scalar green_color(0, 255, 0);

    cv::line(image, projected_points[0], projected_points[1], green_color, 2);
    cv::line(image, projected_points[1], projected_points[2], green_color, 2);
    cv::line(image, projected_points[2], projected_points[3], green_color, 2);
    cv::line(image, projected_points[3], projected_points[0], green_color, 2);

    cv::line(image, projected_points[4], projected_points[5], green_color, 2);
    cv::line(image, projected_points[5], projected_points[6], green_color, 2);
    cv::line(image, projected_points[6], projected_points[7], green_color, 2);
    cv::line(image, projected_points[7], projected_points[4], green_color, 2);

    cv::line(image, projected_points[0], projected_points[4], green_color, 2);
    cv::line(image, projected_points[1], projected_points[5], green_color, 2);
    cv::line(image, projected_points[2], projected_points[6], green_color, 2);
    cv::line(image, projected_points[3], projected_points[7], green_color, 2);
}

cv::Point3f rvecToEulerAngles(const cv::Mat& rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R); // 회전 벡터 -> 회전 행렬

    double sy = std::sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // 특이점(짐벌 락) 확인

    double x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
        y = std::atan2(-R.at<double>(2,0), sy);
        z = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = std::atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    // rad -> deg
    return cv::Point3f(x * 180.0 / CV_PI, y * 180.0 / CV_PI, z * 180.0 / CV_PI);
}