#pragma once

// STD
#include <string>
#include <fstream>
#include <iomanip>

namespace bag_extractor
{

namespace utils
{

//! Template function to convert from base types to string using sstream
template <typename Type>
std::string to_str(const Type &t);

//! Function to get the name of a point cloud file with the number and folder route
std::string get_pc_name(const int counter, const std::string);

//! Function to open the timestamps file.
void openTimestampsFile_(const std::string folder, std::ofstream &outFile);

} // namespace utils

} // namespace bag_extractor