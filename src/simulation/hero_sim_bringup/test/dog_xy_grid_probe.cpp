#include "dog_map/xy_grid.hpp"

#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char** argv) {
    if (argc != 4) return 1;
    try {
        const cv::Mat source = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
        const auto grid = makeDogXYGrid(source, std::stod(argv[2]), std::stod(argv[3]));
        std::cout << std::setprecision(17) << grid.image.cols << ' ' << grid.image.rows
                  << ' ' << grid.resolution << ' ' << grid.size_x << ' ' << grid.size_y
                  << ' ' << (grid.image.data == source.data) << '\n';
        for (int y = 0; y < grid.image.rows; ++y) {
            for (int x = 0; x < grid.image.cols; ++x)
                std::cout << static_cast<int>(grid.image.at<unsigned char>(y, x)) << ' ';
            std::cout << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
