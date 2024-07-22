#ifndef PROMETHEUS_SLAM_MOD_COMMON_H_
#define PROMETHEUS_SLAM_MOD_COMMON_H_

#include <Eigen/Dense>

#include <functional>

namespace prometheus_slam {
namespace mod {

typedef std::function<void(Eigen::Matrix4d, double)> TfPubFunc;
typedef std::function<void(size_t, size_t, double, double, Eigen::Matrix4d)>
    MfPubFunc;
typedef std::function<void(double, double, Eigen::Matrix4d)> LcPubFunc;

typedef int8_t CliId;
}  // namespace mod
}  // namespace prometheus_slam

#endif  
