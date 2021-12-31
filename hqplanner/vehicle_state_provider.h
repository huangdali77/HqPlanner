
// #ifndef HQPLANNER_VEHICLE_STATE_PROVIDER_H_
// #define HQPLANNER_VEHICLE_STATE_PROVIDER_H_

// #include <memory>
// #include <string>

// // #include "modules/canbus/proto/chassis.pb.h"
// // #include "modules/common/macro.h"
// #include "math/box2d.h"
// #include "math/vec2d.h"
// // #include "modules/common/status/status.h"
// #include "for_proto/vehicle_state.h"
// // #include "modules/localization/proto/localization.pb.h"

// /**
//  * @namespace apollo::common
//  * @brief apollo::common
//  */
// namespace hqplanner {
// using hqplanner::forproto::VehicleState;

// /**
//  * @class VehicleStateProvider
//  * @brief The class of vehicle state.
//  *        It includes basic information and computation
//  *        about the state of the vehicle.
//  */
// class VehicleStateProvider {
//  public:
//   /**
//    * @brief Constructor by information of localization and chassis.
//    * @param localization Localization information of the vehicle.
//    * @param chassis Chassis information of the vehicle.
//    */
//   // Status Update(const localization::LocalizationEstimate &localization,
//   //               const canbus::Chassis &chassis);

//   /**
//    * @brief Update VehicleStateProvider instance by protobuf files.
//    * @param localization_file the localization protobuf file.
//    * @param chassis_file The chassis protobuf file
//    */
//   // void Update(const std::string &localization_file,
//   //             const std::string &chassis_file);
//   void Update(const VehicleState &vehicle_state);
//   double timestamp() const;

//   // const localization::Pose &pose() const;
//   // const localization::Pose &original_pose() const;

//   /**
//    * @brief Default destructor.
//    */
//   virtual ~VehicleStateProvider() = default;

//   /**
//    * @brief Get the x-coordinate of vehicle position.
//    * @return The x-coordinate of vehicle position.
//    */
//   double x() const;

//   /**
//    * @brief Get the y-coordinate of vehicle position.
//    * @return The y-coordinate of vehicle position.
//    */
//   double y() const;

//   /**
//    * @brief Get the z coordinate of vehicle position.
//    * @return The z coordinate of vehicle position.
//    */
//   double z() const;

//   double kappa() const;

//   /**
//    * @brief Get the vehicle roll angle.
//    * @return The euler roll angle.
//    */
//   double roll() const;

//   /**
//    * @brief Get the vehicle pitch angle.
//    * @return The euler pitch angle.
//    */
//   double pitch() const;

//   /**
//    * @brief Get the vehicle yaw angle.
//    *  As of now, use the heading instead of yaw angle.
//    *  Heading angle with East as zero, yaw angle has North as zero
//    * @return The euler yaw angle.
//    */
//   double yaw() const;

//   /**
//    * @brief Get the heading of vehicle position, which is the angle
//    *        between the vehicle's heading direction and the x-axis.
//    * @return The angle between the vehicle's heading direction
//    *         and the x-axis.
//    */
//   double heading() const;

//   /**
//    * @brief Get the vehicle's linear velocity.
//    * @return The vehicle's linear velocity.
//    */
//   double linear_velocity() const;

//   /**
//    * @brief Get the vehicle's angular velocity.
//    * @return The vehicle's angular velocity.
//    */
//   double angular_velocity() const;

//   /**
//    * @brief Get the vehicle's linear acceleration.
//    * @return The vehicle's linear acceleration.
//    */
//   double linear_acceleration() const;

//   /**
//    * @brief Set the vehicle's linear velocity.
//    * @param linear_velocity The value to set the vehicle's linear velocity.
//    */
//   void set_linear_velocity(const double linear_velocity);

//   /**
//    * @brief Estimate future position from current position and heading,
//    *        along a period of time, by constant linear velocity,
//    *        linear acceleration, angular velocity.
//    * @param t The length of time period.
//    * @return The estimated future position in time t.
//    */
//   math::Vec2d EstimateFuturePosition(const double t) const;

//   const VehicleState &vehicle_state() const;

//   void set_vehicle_config(const double x, const double y, const double
//   heading);

//  public:
//   static VehicleState vehicle_state_;

//  private:
//   // bool ConstructExceptLinearVelocity(
//   //     const localization::LocalizationEstimate &localization);

//   // VehicleState vehicle_state_;
//   // localization::LocalizationEstimate original_localization_;

//   // DECLARE_SINGLETON(VehicleStateProvider);
// };
// // ======================函数实现===================================
// void VehicleStateProvider::Update(const VehicleState &vehicle_state) {
//   vehicle_state_ = vehicle_state;
// }
// double VehicleStateProvider::x() const { return vehicle_state_.x; }

// double VehicleStateProvider::y() const { return vehicle_state_.y; }

// double VehicleStateProvider::z() const { return vehicle_state_.z; }

// double VehicleStateProvider::roll() const { return vehicle_state_.roll; }

// double VehicleStateProvider::pitch() const { return vehicle_state_.pitch; }

// double VehicleStateProvider::yaw() const { return vehicle_state_.yaw; }

// double VehicleStateProvider::heading() const { return vehicle_state_.heading;
// }

// double VehicleStateProvider::kappa() const { return vehicle_state_.kappa; }

// double VehicleStateProvider::linear_velocity() const {
//   return vehicle_state_.linear_velocity;
// }

// double VehicleStateProvider::angular_velocity() const {
//   return vehicle_state_.angular_velocity;
// }

// double VehicleStateProvider::linear_acceleration() const {
//   return vehicle_state_.linear_acceleration;
// }

// double VehicleStateProvider::timestamp() const {
//   return vehicle_state_.timestamp;
// }

// // const localization::Pose &VehicleStateProvider::pose() const {
// //   return vehicle_state_.pose();
// // }

// // const localization::Pose &VehicleStateProvider::original_pose() const {
// //   return original_localization_.pose();
// // }

// void VehicleStateProvider::set_linear_velocity(const double linear_velocity)
// {
//   vehicle_state_.linear_velocity = linear_velocity;
// }

// const VehicleState &VehicleStateProvider::vehicle_state() const {
//   return vehicle_state_;
// }

// void VehicleStateProvider::set_vehicle_config(const double x, const double y,
//                                               const double heading) {
//   vehicle_state_.x = x;
//   vehicle_state_.y = y;
//   vehicle_state_.heading = heading;
// }

// }  // namespace hqplanner

// #endif  // MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_PROVIDER_H_