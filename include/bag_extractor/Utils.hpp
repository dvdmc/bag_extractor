#pragma once

// STD
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>

namespace bag_extractor
{

namespace utils
{

//! Function to get the name of a file with the number and folder route
std::string get_file_name(const int counter, const std::string folder, const double timestamp, const std::string extension);

//! Function to get several strings from a string list and a separator
std::vector<std::string> split_strings(std::string const &str, char const &delimiter);

} // namespace utils

} // namespace bag_extractor