#include <cmath>

#include "imu_propagation/imu_propagator.hpp"

namespace imu_propagation
{

ImuPropagator::ImuPropagator() = default;

const ImuState &ImuPropagator::state() const
{
    return state_;
}

void ImuPropagator::reset()
{
    state_ = ImuState{};
}

bool ImuPropagator::reset(const ImuState &initial_state)
{
    if (!std::isfinite(initial_state.timestamp) ||
        !initial_state.position_wb.allFinite() ||
        !initial_state.velocity_wb.allFinite() ||
        !initial_state.orientation_wb.coeffs().allFinite() ||
        !initial_state.accelerometer_bias.allFinite() ||
        !initial_state.gyroscope_bias.allFinite()) {
        return false;
    }

    const double orientation_norm = initial_state.orientation_wb.norm();

    if (!std::isfinite(orientation_norm) || orientation_norm < 1.0e-12) {
        return false;
    }
    state_ = initial_state;
    state_.orientation_wb.normalize();

    return true;
}

Eigen::Quaterniond ImuPropagator::delta_quaternion(
    const Eigen::Vector3d &angular_velocity, double dt)
{
    const Eigen::Vector3d rotation_vector = angular_velocity * dt;

    const double angle = rotation_vector.norm();

    Eigen::Quaterniond delta_q;

    if (angle < 1.0e-8) {
        delta_q.w() = 1.0;
        delta_q.vec() = 0.5 * rotation_vector;
    } else {
        const double half_angle = 0.5 * angle;
        const double scale = std::sin(half_angle) / angle;

        delta_q.w() = std::cos(half_angle);
        delta_q.vec() = scale * rotation_vector;
    }

    delta_q.normalize();

    return delta_q;
}


bool ImuPropagator::propagate(
    const ImuMeasurement &previous_measurement,
    const ImuMeasurement &current_measurement)
{
    const double dt = current_measurement.timestamp - previous_measurement.timestamp;

    if (!std::isfinite(dt) || dt <= 0.0) {
        return false;
    }

    if (!previous_measurement.angular_velocity.allFinite() ||
		!current_measurement.angular_velocity.allFinite() ||
		!previous_measurement.acceleration.allFinite() ||
		!current_measurement.acceleration.allFinite()) {
		return false;
	}

    const Eigen::Quaterniond orientation_0 = state_.orientation_wb;
    
    const Eigen::Vector3d average_angular_velocity = 
        0.5 * (previous_measurement.angular_velocity + current_measurement.angular_velocity) - state_.gyroscope_bias;
    
    Eigen::Quaterniond orientation_1 = orientation_0 * delta_quaternion(average_angular_velocity, dt);

    orientation_1.normalize();

    const Eigen::Vector3d acceleration_world_0 = 
        orientation_0 * (previous_measurement.acceleration - state_.accelerometer_bias) + gravity_world_;

    const Eigen::Vector3d acceleration_world_1 = 
        orientation_1 * (current_measurement.acceleration - state_.accelerometer_bias) + gravity_world_;

    const Eigen::Vector3d average_acceleration_world = 
        0.5 * (acceleration_world_0 + acceleration_world_1);
    
    const Eigen::Vector3d new_position = 
        state_.position_wb + state_.velocity_wb * dt + 0.5 * average_acceleration_world * dt * dt;

    const Eigen::Vector3d new_velocity = 
        state_.velocity_wb + average_acceleration_world * dt;

    
    state_.position_wb = new_position;
    state_.velocity_wb = new_velocity;
    state_.orientation_wb = orientation_1;
    state_.timestamp = current_measurement.timestamp;

    return true;
}


}   // namespace imu_propagation