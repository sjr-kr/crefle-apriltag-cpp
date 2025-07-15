#include "generate_markers.hpp"
#include <iostream>
#include <string>
#include <sstream>

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