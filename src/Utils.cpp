#include "bag_extractor/Utils.hpp"

namespace bag_extractor
{

namespace utils
{

template <typename Type>
std::string to_str(const Type &t)
{
    std::ostringstream os;
    os << t;
    return os.str();
}

std::string get_pc_name(const int number, const std::string folder)
{

    std::string number_str = utils::to_str(number);

    std::string filename = ".bin";
    filename = number_str + filename;

    std::ostringstream os;

    os << std::setw(14) << std::setfill('0') << filename;
    filename = os.str();

    filename = "./" + folder + filename;

    return filename;
}

void openTimestampsFile_(const std::string folder, std::ofstream &outFile)
{

    std::string filename = "timestamps.txt";

    filename = "./" + folder + filename;

    outFile.open(filename.c_str(), std::ios::out);

}

} // namespace utils

} // namespace bag_extractor