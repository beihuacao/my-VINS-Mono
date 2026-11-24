#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace imu_propagation
{

struct ImuMeasurement
{
    double timestamp = 0.0;

    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct ImuState
{
    double timestamp = 0.0;

    Eigen::Vector3d position_wb = Eigen::Vector3d::Zero();

    Eigen::Vector3d velocity_wb = Eigen::Vector3d::Zero();

    Eigen::Quaterniond orientation_wb = Eigen::Quaterniond::Identity();

    Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();

    Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();
};

class ImuPropagator
{
public:
    ImuPropagator();

    const ImuState &state() const;

    void reset();

    bool reset(const ImuState &initial_state);

    bool propagate(
        const ImuMeasurement &previous_measurement,
        const ImuMeasurement &current_measurement);


private:

    static Eigen::Quaterniond delta_quaternion(
        const Eigen::Vector3d &angular_velocity, double dt);

    ImuState state_;

    Eigen::Vector3d gravity_world_ {
        0.0,
        0.0,
        -9.81};

};


}   // namespace imu_propagation