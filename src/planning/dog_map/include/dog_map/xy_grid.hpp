#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <stdexcept>

struct DogXYGrid {
    cv::Mat image;
    double resolution;
    double size_x;
    double size_y;
};

// PGM rows start at the top, but DOG indices start at the map's lower left.
// Coarsen from that lower left so neither the origin nor observed points move.
inline DogXYGrid makeDogXYGrid(const cv::Mat& source, double map_resolution,
                              double requested_resolution) {
    if (!std::isfinite(map_resolution) || map_resolution <= 0.0 ||
        !std::isfinite(requested_resolution) || requested_resolution < 0.0)
        throw std::invalid_argument("DOG resolutions must be finite; map resolution must be positive and resolution_xy nonnegative");
    if (source.empty() || source.type() != CV_8UC1)
        throw std::invalid_argument("DOG requires a nonempty grayscale PGM");
    const double ratio = requested_resolution == 0.0
                             ? 1.0 : requested_resolution / map_resolution;
    const double rounded = std::round(ratio);
    if (rounded < 1.0 || rounded > std::numeric_limits<int>::max() ||
        std::abs(ratio - rounded) > 1e-9)
        throw std::invalid_argument("resolution_xy must be 0 (follow PGM) or an integer multiple of the PGM resolution");
    const int factor = static_cast<int>(rounded);
    DogXYGrid result{source, map_resolution * factor,
                     source.cols * map_resolution, source.rows * map_resolution};
    if (factor == 1) return result;  // Preserve the original path exactly.

    const int cols = 1 + (source.cols - 1) / factor;
    const int rows = 1 + (source.rows - 1) / factor;
    result.image = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255));
    for (int y = 0; y < rows; ++y) {
        const int end_row = source.rows - y * factor;
        const int begin_row = std::max(0, end_row - factor);
        for (int x = 0; x < cols; ++x) {
            unsigned char darkest = 255;
            for (int sy = begin_row; sy < end_row; ++sy) {
                const auto* pixels = source.ptr<unsigned char>(sy);
                const int end_col = std::min(source.cols, (x + 1) * factor);
                for (int sx = x * factor; sx < end_col; ++sx)
                    darkest = std::min(darkest, pixels[sx]);
            }
            // Keep black/unknown pixels rather than averaging away thin walls.
            result.image.at<unsigned char>(rows - 1 - y, x) = darkest;
        }
    }
    return result;
}
