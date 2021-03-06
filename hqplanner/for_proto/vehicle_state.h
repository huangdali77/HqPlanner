#ifndef HQPLANNER_FORPROTO_VEHICLESTATE_H_
#define HQPLANNER_FORPROTO_VEHICLESTATE_H_
namespace hqplanner {
namespace forproto {
struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double timestamp = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double heading = 0.0;
  double kappa = 0.0;
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  double linear_acceleration = 0.0;
  //  public:
  //   void SetX(double x) { x_ = x; }

  //   void SetY(double y) { y_ = y; }

  //   void SetHeading(double heading) { heading_ = heading; }

  //   void SetV(double v) { v_ = v; }
  //   void SetVX(double vx) { v_x_ = vx; }

  //   void SetVY(double vy) { v_y_ = vy; }

  //   void SetYawD(double yaw_d) { yaw_d_ = yaw_d; }

  //   void SetAX(double ax) { a_x_ = ax; }

  //   void SetAY(double ay) { a_y_ = ay; }

  //   void SetYawDD(double yaw_dd) { yaw_dd_ = yaw_dd; }

  //   void SetTimeStamp(double time_stamp) { time_stamp_ = time_stamp; }

  //   // ==============================
  //   double GetX() { return x_; }

  //   double GetY() { return y_; }

  //   double GetHeading() { return heading_; }

  //   double GetV() { return v_; }

  //   double GetVX() { return v_x_; }

  //   double GetVY() { return v_y_; }

  //   double GetYawD() { return yaw_d_; }

  //   double GetAX() { return a_x_; }

  //   double GetAY() { return a_y_; }

  //   double GetYawDD() { return yaw_dd_; }

  //   double GetYawDD() { return yaw_dd_; }

  //   double SetTimeStamp() { return time_stamp_; }

  //  private:
  //   //  位姿
  //   double x_;
  //   double y_;
  //   double heading_;
  //   // 速度
  //   double v_;
  //   double v_x_;
  //   double v_y_;
  //   double yaw_d_;
  //   // 加速度
  //   double a_x_;
  //   double a_y_;
  //   double yaw_dd_;
  //   // timestamp
  //   double time_stamp_;

  // ===========================================

  // apollo.canbus.Chassis.GearPosition gear = 13;
  // apollo.canbus.Chassis.DrivingMode driving_mode = 14;
  // apollo.localization.Pose pose = 15;
};
}  // namespace forproto
}  // namespace hqplanner

#endif