#ifndef GENERATE_MARKERS_HPP
#define GENERATE_MARKERS_HPP

#include <vector>
#include <string>

void generate_apriltags(const std::vector<int>& ids, int image_size, const std::string& output_dir);

#endif // GENERATE_MARKERS_HPP