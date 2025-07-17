#include "generate_markers.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
// 다른 dict들은 필요에 따라 주석 해제하거나 추가
// #include <apriltag/tag25h9.h>
// #include <apriltag/tag16h5.h>
// #include <apriltag/tagCircle21h7.h>
// #include <apriltag/tagCircle49h12.h>
// #include <apriltag/tagCircle49h12.h>
// #include <apriltag/tagCustom48h12.h>
// #include <apriltag/tagStandard41h12.h>
// #include <apriltag/tagStandard52h13.h>

namespace fs = std::filesystem;

void generate_apriltags(const std::vector<int>& ids, int image_size, const std::string& output_dir) {
    // 디렉토리 없으면 생성
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }

    // dict 선택; ex) tag36h11
    apriltag_family_t *tf = tag36h11_create();
    if (!tf) {
        std::cerr << "Error: AprilTag 패밀리 생성 실패!" << std::endl;
        return;
    }

    // 마커 생성
    for (int id : ids) {
        if (id < 0 || id >= tf->ncodes) {
            std::cerr << "Warning: AprilTag ID " << id << " 이(가) 해당 패밀리 범위 밖! 건너뛰는 중..." << std::endl;
            continue;
        }

        // AprilTag ID에 해당하는 마커 불러오기
        image_u8_t *im = apriltag_to_image(tf, id);
        if (!im) {
            std::cerr << "Error: ID " << id << "에 해당하는 마커 이미지 생성 실패! 건너뛰는 중..." << std::endl;
            continue;
        }

        // im->stride : 이미지 이동 사이즈
        cv::Mat marker_img(im->height, im->width, CV_8UC1, im->buf, im->stride);

        // 사이즈 변경
        cv::Mat resized_img;
        cv::resize(marker_img, resized_img, cv::Size(image_size, image_size), 0, 0, cv::INTER_NEAREST);

        // 마커 저장 디렉토리
        std::string filename = output_dir + "/tag" + std::to_string(id) + ".png";
        cv::imwrite(filename, resized_img);
        std::cout << "생성된 파일: " << filename << std::endl;

        image_u8_destroy(im);
    }

    tag36h11_destroy(tf);
}

int main() {
    std::vector<int> ids_to_generate;
    std::string input_line;
    int image_size = 500; // 500x500 pixel (standard)
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