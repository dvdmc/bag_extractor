#include "bag_extractor/Utils.hpp"
#include <iostream>
namespace bag_extractor
{

namespace utils
{

std::string get_file_name(const int number, const std::string folder, const double timestamp, const std::string extension)
{

    std::string number_str = std::to_string(number);
    std::string timestamp_str = std::to_string(timestamp);

    std::string filename = extension;
    filename = number_str + "_" + timestamp_str + filename;

    std::ostringstream os;

    os << std::setw(32) << std::setfill('0') << filename;
    filename = os.str();

    filename = "./" + folder + filename;

    return filename;
}

std::vector<std::string> split_strings(std::string const &str, char const &delimiter)
{
    std::stringstream ss(str);
    std::string item;
    std::vector<std::string> output;
    while(std::getline(ss, item, delimiter))
    {
        std::cout << "Introduced item: " << item.c_str() << std::endl;
        output.push_back(item);
    }
    return output;
}

} // namespace utils

} // namespace bag_extractor