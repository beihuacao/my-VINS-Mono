#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "imu_propagation/imu_propagator.hpp"

namespace
{

constexpr double kPi =
	3.14159265358979323846;

}  // namespace

TEST(ImuPropagatorTest, RotatesNinetyDegreesAroundZ)
{
	imu_propagation::ImuPropagator propagator;

	imu_propagation::ImuMeasurement previous;
	previous.timestamp = 0.0;
	previous.angular_velocity =
		Eigen::Vector3d(
			0.0,
			0.0,
			kPi / 2.0);

	imu_propagation::ImuMeasurement current;
	current.timestamp = 1.0;
	current.angular_velocity =
		Eigen::Vector3d(
			0.0,
			0.0,
			kPi / 2.0);

	ASSERT_TRUE(
		propagator.propagate(previous, current));

	const Eigen::Quaterniond expected_orientation{
		Eigen::AngleAxisd(
			kPi / 2.0,
			Eigen::Vector3d::UnitZ())};

	EXPECT_NEAR(
		propagator.state().timestamp,
		1.0,
		1.0e-12);

	EXPECT_NEAR(
		propagator.state().orientation_wb.angularDistance(
			expected_orientation),
		0.0,
		1.0e-12);
}

TEST(ImuPropagatorTest, RejectsNonIncreasingTimestamp)
{
	imu_propagation::ImuPropagator propagator;

	imu_propagation::ImuMeasurement previous;
	previous.timestamp = 1.0;

	imu_propagation::ImuMeasurement current;
	current.timestamp = 1.0;

	EXPECT_FALSE(
		propagator.propagate(previous, current));

	EXPECT_NEAR(
		propagator.state().timestamp,
		0.0,
		1.0e-12);

	EXPECT_NEAR(
		propagator.state().orientation_wb.angularDistance(
			Eigen::Quaterniond::Identity()),
		0.0,
		1.0e-12);
}

TEST(ImuPropagatorTest, RemovesGyroscopeBias)
{
	imu_propagation::ImuPropagator propagator;

	imu_propagation::ImuState initial_state;
	initial_state.gyroscope_bias =
		Eigen::Vector3d(
			0.0,
			0.0,
			kPi / 2.0);

	ASSERT_TRUE(propagator.reset(initial_state));

	imu_propagation::ImuMeasurement previous;
	previous.timestamp = 0.0;
	previous.angular_velocity =
		Eigen::Vector3d(
			0.0,
			0.0,
			kPi / 2.0);

	imu_propagation::ImuMeasurement current;
	current.timestamp = 1.0;
	current.angular_velocity =
		previous.angular_velocity;

	ASSERT_TRUE(
		propagator.propagate(previous, current));

	EXPECT_NEAR(
		propagator.state().orientation_wb.angularDistance(
			Eigen::Quaterniond::Identity()),
		0.0,
		1.0e-12);
}

TEST(ImuPropagatorTest, AccumulatesNinetyDegreesAtTwoHundredHertz)
{
	imu_propagation::ImuPropagator propagator;

	constexpr int kSampleCount = 200;
	constexpr double kDt = 0.005;

	imu_propagation::ImuMeasurement previous;
	previous.timestamp = 0.0;
	previous.angular_velocity =
		Eigen::Vector3d(
			0.0,
			0.0,
			kPi / 2.0);

	for (int i = 1; i <= kSampleCount; ++i) {
		imu_propagation::ImuMeasurement current;

		current.timestamp =
			static_cast<double>(i) * kDt;

		current.angular_velocity =
			previous.angular_velocity;

		ASSERT_TRUE(
			propagator.propagate(
				previous,
				current));

		previous = current;
	}

	const Eigen::Quaterniond expected_orientation{
		Eigen::AngleAxisd(
			kPi / 2.0,
			Eigen::Vector3d::UnitZ())};

	EXPECT_NEAR(
		propagator.state().orientation_wb.angularDistance(
			expected_orientation),
		0.0,
		1.0e-10);

	EXPECT_NEAR(
		propagator.state().orientation_wb.norm(),
		1.0,
		1.0e-12);
}

TEST(ImuPropagatorTest, RejectsZeroInitialQuaternion)
{
	imu_propagation::ImuPropagator propagator;

	imu_propagation::ImuState invalid_state;
	invalid_state.orientation_wb =
		Eigen::Quaterniond(
			0.0,
			0.0,
			0.0,
			0.0);

	EXPECT_FALSE(
		propagator.reset(invalid_state));

	EXPECT_NEAR(
		propagator.state().orientation_wb.angularDistance(
			Eigen::Quaterniond::Identity()),
		0.0,
		1.0e-12);
}