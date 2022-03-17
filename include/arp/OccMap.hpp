#pragma once

#include <string>

#include <Eigen/Eigen>
#include <opencv2/core/mat.hpp>

namespace arp {

Eigen::Vector3i round(const Eigen::Vector3d& v);
Eigen::Vector3i modulo(const Eigen::Vector3i& v, int m);

class OccMap {
    cv::Mat map_;
    int sizes_[3];
    char* mapData_;
    Eigen::Vector3i offset_; ///< offset in grid coords where the world origin is
    double rasterSize_;

public:
    explicit OccMap(const std::string& filename);

    OccMap(const OccMap& other, int factor);

    ~OccMap();

    char isOccupiedGrid(const Eigen::Vector3i& gridPos) const;

    bool isOccupiedWorld(const Eigen::Vector3d& worldPos) const;

    bool isOccupiedGridRegion(const Eigen::Vector3i& gridPosA, const Eigen::Vector3i& gridPosB) const;

    Eigen::Vector3i toGridPos(const Eigen::Vector3d& worldPos) const;

    Eigen::Vector3d toWorldPos(const Eigen::Vector3i& gridPos) const;

    const int* sizes() const {
        return sizes_;
    }

    Eigen::Vector3d worldOffset() const {
        return -offset_.cast<double>() * rasterSize_ - Eigen::Vector3d::Constant(rasterSize_ / 2);
    }

    double rasterSize() const {
        return rasterSize_;
    }
};

}