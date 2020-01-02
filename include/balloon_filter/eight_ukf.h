#include <mrs_lib/ukf.h>

namespace balloon_filter
{
  namespace ukf
  {
      /* constexpr int kf_n_states = 10; */
    constexpr int n_states = 5;
    constexpr int n_inputs = 4;
    constexpr int n_measurements = 3;
    using UKF = mrs_lib::UKF<n_states, n_inputs, n_measurements>;

    // This function implements the state transition
    UKF::x_t tra_model_f(const UKF::x_t& x, [[maybe_unused]] const UKF::u_t& u, const double dt);

    // This function implements the observation generation from a state
    UKF::z_t obs_model_f(const UKF::x_t& x);

    // indices of the state interpretations
    enum
    {
      x_x = 0, // 3D x-coordinate of the ball position
      x_y,     // 3D y-coordinate of the ball position
      x_z,     // 3D z-coordinate of the ball position
      x_yaw,   // yaw of the MAV in the eight-plane
      /* x_s,     // the ball speed */
      x_c,     // curvature of the MAV trajectory in the eight-plane
    };

    // indices of the input interpretations
    enum
    {
      u_s = 0, // the ball speed
      u_qw,    // w element of quaterion, defining rotation from world frame to the eight-plane frame
      u_qx,    // x element of quaterion, defining rotation from world frame to the eight-plane frame
      u_qy,    // y element of quaterion, defining rotation from world frame to the eight-plane frame
      u_qz,    // z element of quaterion, defining rotation from world frame to the eight-plane frame
    };

    // indices of the measurement interpretations
    enum
    {
      z_x = 0, // 3D x-coordinate of the ball position
      z_y,     // 3D y-coordinate of the ball position
      z_z,     // 3D z-coordinate of the ball position
    };
  }
}
