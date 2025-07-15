#include "generate_markers.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
// 다른 딕셔너리들은 필요에 따라 주석 해제하거나 추가하세요.
// #include <apriltag/tag25h9.h>
// #include <apriltag/tag16h5.h>
// #include <apriltag/tagCircle21h7.h>
// #include <apriltag/tagCircle49h12.h>
// #include <apriltag/tagCircle49h12.h>
// #include <apriltag/tagCustom48h12.h>
// #include <apriltag/tagStandard41h12.h>
// #include <apriltag/tagStandard52h13.h>

// OpenCV를 사용하여 이미지 저장
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

void generate_apriltags(const std::vector<int>& ids, int image_size, const std::string& output_dir) {
    // 출력 디렉토리가 없으면 생성합니다.
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }

    // 사용할 AprilTag 딕셔너리를 선택합니다. 여기서는 tag36h11을 사용합니다.
    apriltag_family_t *tf = tag36h11_create();
    if (!tf) {
        std::cerr << "Error: Could not create AprilTag family." << std::endl;
        return;
    }

    // 마커 생성
    for (int id : ids) {
        if (id < 0 || id >= tf->ncodes) {
            std::cerr << "Warning: AprilTag ID " << id << " is out of range for this family. Skipping." << std::endl;
            continue;
        }

        // AprilTag 라이브러리에서 마커 비트맵을 가져옵니다.
        image_u8_t *im = apriltag_to_image(tf, id);
        if (!im) {
            std::cerr << "Error: Could not generate image for ID " << id << ". Skipping." << std::endl;
            continue;
        }

        // OpenCV Mat으로 변환 시 im->stride를 사용하여 올바른 메모리 레이아웃을 지정합니다.
        cv::Mat marker_img(im->height, im->width, CV_8UC1, im->buf, im->stride);

        // 원하는 크기로 리사이즈
        cv::Mat resized_img;
        cv::resize(marker_img, resized_img, cv::Size(image_size, image_size), 0, 0, cv::INTER_NEAREST);

        // 파일 저장 경로
        std::string filename = output_dir + "/tag" + std::to_string(id) + ".png";
        cv::imwrite(filename, resized_img);
        std::cout << "Generated " << filename << std::endl;

        image_u8_destroy(im);
    }

    // AprilTag 딕셔너리 자원 해제
    tag36h11_destroy(tf);
}

int main() {
    std::vector<int> ids_to_generate;
    std::string input_line;
    int image_size = 500; // 기본 이미지 크기를 500x500 픽셀로 설정
    std::string output_dir = "generated_markers";

    std::cout << "Enter AprilTag IDs to generate (space-separated, e.g., 0 1 2 3 10): ";
    std::getline(std::cin, input_line);
    std::stringstream ss(input_line);
    int id;
    while (ss >> id) {
        ids_to_generate.push_back(id);
    }

    std::cout << "Generating AprilTags with image size " << image_size << "x" << image_size << " pixels." << std::endl;
    std::cout << "(You can modify 'image_size' in src/generate_markers_main.cpp if needed.)" << std::endl;

    generate_apriltags(ids_to_generate, image_size, output_dir);

    return 0;
}