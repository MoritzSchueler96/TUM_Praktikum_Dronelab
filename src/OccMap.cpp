
#include "arp/OccMap.hpp"

#include <fstream>
#include <iostream>

namespace arp {

/// round the elements of a Vector3d to the nearest integers
/// unlike cast<int>() witch rounds towards zero!
Eigen::Vector3i round(const Eigen::Vector3d& v) {
    return {
        static_cast<int>(std::round(v[0])),
        static_cast<int>(std::round(v[1])),
        static_cast<int>(std::round(v[2]))};
}

Eigen::Vector3i modulo(const Eigen::Vector3i& v, int m) {
    return {
        v[0] % m,
        v[1] % m,
        v[2] % m};
}

OccMap::OccMap(const std::string& filename) : rasterSize_(0.1) {
    std::cout << "Loading OccMap " << filename << std::endl;

    std::ifstream mapFile(filename, std::ios::in | std::ios::binary);
    if (!mapFile.is_open()) {
        throw std::runtime_error("Could not open OccMap");
    }

    if (!mapFile.read((char*)sizes_, 3 * sizeof(int))) {
        throw std::runtime_error("Could not read sizes of OccMap");
    }

    mapData_ = new char[sizes_[0] * sizes_[1] * sizes_[2]];  // don’t forget \
    to delete[] in the end!
    if (!mapFile.read((char*)mapData_, sizes_[0] * sizes_[1] * sizes_[2])) {
        throw std::runtime_error("Could not read OccMap data");
    }
    mapFile.close();

    // now wrap it with a cv::Mat for easier access:
    map_ = cv::Mat(3, sizes_, CV_8SC1, mapData_);

    offset_ = Eigen::Vector3i((sizes_[0] - 1) / 2, (sizes_[1] - 1) / 2, (sizes_[2] - 1) / 2);
}

OccMap::OccMap(const OccMap& other, int factor) : rasterSize_(other.rasterSize_ * factor) {
    std::cout << "Starting to upsample..." << std::endl;
    sizes_[0] = other.sizes_[0] / factor - 1;
    sizes_[1] = other.sizes_[1] / factor - 1;
    sizes_[2] = other.sizes_[2] / factor - 1;

    mapData_ = new char[sizes_[0] * sizes_[1] * sizes_[2]];
    map_ = cv::Mat(3, sizes_, CV_8SC1, mapData_);

    auto origin = modulo(other.offset_ + Eigen::Vector3i::Constant((factor - 1) / 2), factor);

    for (int x = 0; x < sizes_[0]; x++) {
        for (int y = 0; y < sizes_[1]; y++) {
            for (int z = 0; z < sizes_[2]; z++) {
                bool occupied = other.isOccupiedGridRegion(
                    origin + Eigen::Vector3i{factor * x, factor * y, factor * z},
                    origin + Eigen::Vector3i{factor * (x + 1), factor * (y + 1), factor * (z + 1)});
                map_.at<char>(x, y, z) = occupied ? 5 : -5;
            }
        }
    }

    offset_ = Eigen::Vector3i((sizes_[0] - 1) / 2, (sizes_[1] - 1) / 2, (sizes_[2] - 1) / 2);
    std::cout << "Done upsampling..." << std::endl;
}

OccMap::~OccMap() {
    map_ = cv::Mat();
    delete[] mapData_;
}

char OccMap::isOccupiedGrid(const Eigen::Vector3i& gridPos) const {
    if (gridPos[0] < 0 || gridPos[1] < 0 || gridPos[2] < 0) {
        std::cout << "OccMap: out of bounds A!" << std::endl;
        return 127; // true;
    }
    if (gridPos[0] >= sizes_[0] || gridPos[1] >= sizes_[1] || gridPos[2] >= sizes_[2]) {
        std::cout << "OccMap: out of bounds B!" << std::endl;
        return 127; // true;
    }

    char data = map_.at<char>(gridPos[0], gridPos[1], gridPos[2]);
    return data; //  >= 0;
}

bool OccMap::isOccupiedWorld(const Eigen::Vector3d& worldPos) const {
    return isOccupiedGrid(toGridPos(worldPos)) >= 0;
}

bool OccMap::isOccupiedGridRegion(const Eigen::Vector3i& gridPosA, const Eigen::Vector3i& gridPosB) const {
    if (Eigen::Vector3i(gridPosB - gridPosA).minCoeff() < 0) {
        throw std::runtime_error("Invalid points for isOccupiedGridRegion. B must be greater or equal in all directions than A");
    }
    for (int x = gridPosA[0]; x < gridPosB[0]; x++) {
        for (int y = gridPosA[1]; y < gridPosB[1]; y++) {
            for (int z = gridPosA[2]; z < gridPosB[2]; z++) {
                if (isOccupiedGrid({x, y, z})>=0) {
                    return true;
                }
            }
        }
    }
    return false;
}

Eigen::Vector3i OccMap::toGridPos(const Eigen::Vector3d& worldPos) const {
    return round(worldPos / rasterSize_) + offset_;
}

Eigen::Vector3d OccMap::toWorldPos(const Eigen::Vector3i& gridPos) const {
    return (gridPos - offset_).cast<double>() * rasterSize_;
}

}  // namespace arp
