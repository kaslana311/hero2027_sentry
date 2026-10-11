#pragma once

#include <cmath>
#include <stdexcept>

// Output classification only: never erase the underlying occupied voxels.
// min_z is absolute map Z, before the node's artificial cloud-Z translation.
class HighColumnFilter {
 public:
    HighColumnFilter(bool enabled = false, double min_z = 0.30,
                     double cloud_z_offset = 0.30)
        : enabled_(enabled), min_z_(min_z), cloud_z_offset_(cloud_z_offset) {
        if (!std::isfinite(min_z_) || min_z_ < 0.0 ||
            !std::isfinite(cloud_z_offset_)) {
            throw std::invalid_argument(
                "high_column_min_z must be finite and nonnegative; cloud Z offset must be finite");
        }
    }

    bool ignores(int lowest_occupied_layer, double resolution_z) const {
        if (!enabled_ || lowest_occupied_layer < 0) return false;
        // Use the voxel's LOWER face, not its centre, so a straddling voxel is
        // retained. Do not use fake_layer_z: its historical 0.35 m reference
        // differs from the actual 0.30/0.26 m cloud translations.
        const double lower_face_map_z =
            lowest_occupied_layer * resolution_z - cloud_z_offset_;
        return lower_face_map_z > min_z_ + 1e-9;
    }

 private:
    bool enabled_;
    double min_z_;
    double cloud_z_offset_;
};
