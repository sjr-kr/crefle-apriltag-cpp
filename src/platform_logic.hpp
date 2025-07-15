#ifndef PLATFORM_LOGIC_HPP
#define PLATFORM_LOGIC_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

// 포즈 정보(회전, 변환, 2D 중심)를 저장하는 구조체
struct Pose {
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Point2d center;
};

// 플랫폼 좌표계의 변환을 계산합니다.
bool getPlatformTransform(
    const std::map<int, Pose>& detected_markers,
    const std::vector<int>& platform_ids,
    double platform_side_length,
    cv::Mat& platform_rvec,
    cv::Mat& platform_tvec,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs
);

// 플랫폼 좌표계에 대한 객체 마커의 상대 위치를 계산합니다.
cv::Point3f getRelativePosition(
    const Pose& object_pose,
    const cv::Mat& platform_rvec,
    const cv::Mat& platform_tvec
);

// 이미지에 좌표축을 그리는 헬퍼 함수
void drawAxes(cv::InputOutputArray image, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
              const cv::Mat& rvec, const cv::Mat& tvec, float length);

// 객체 마커 주변에 큐브를 그리는 함수
void drawCube(cv::InputOutputArray image, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
              const cv::Mat& rvec, const cv::Mat& tvec, float marker_size);

// 회전 벡터(rvec)를 오일러 각(Roll, Pitch, Yaw)으로 변환하는 함수
cv::Point3f rvecToEulerAngles(const cv::Mat& rvec);

#endif // PLATFORM_LOGIC_HPP
