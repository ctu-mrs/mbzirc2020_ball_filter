#include <mrs_lib/ukf.h>

namespace ball_filter
{
  namespace ukf
  {
    // indices of the state interpretations
    namespace x
    {
      enum
      {
        x = 0, // 3D x-coordinate of the ball position
        y,     // 3D y-coordinate of the ball position
        z,     // 3D z-coordinate of the ball position
        yaw,   // yaw of the MAV in the eight-plane
        /* s,     // the ball speed */
        c,     // curvature of the MAV trajectory in the eight-plane
        LENGTH
      };
    }

    // indices of the input interpretations
    namespace u
    {
      enum
      {
        s = 0, // the ball speed
        qw,    // w element of quaterion, defining rotation from world frame to the eight-plane frame
        qx,    // x element of quaterion, defining rotation from world frame to the eight-plane frame
        qy,    // y element of quaterion, defining rotation from world frame to the eight-plane frame
        qz,    // z element of quaterion, defining rotation from world frame to the eight-plane frame
        LENGTH
      };
    }

    namespace z
    {
      // indices of the measurement interpretations
      enum
      {
        c = 0, // curvature in the 2D plane
        x = 0, // 3D x-coordinate of the ball position
        y,     // 3D y-coordinate of the ball position
        z,     // 3D z-coordinate of the ball position
        yaw,   // yaw in the 2D plane
        LENGTH
      };
    }

    constexpr int n_states = x::LENGTH;
    constexpr int n_inputs = u::LENGTH;
    constexpr int n_measurements = -1; // number of measurements is variable
    using UKF = mrs_lib::UKF<n_states, n_inputs, n_measurements>;

    // This function implements the state transition
    UKF::x_t tra_model_f(const UKF::x_t& x, [[maybe_unused]] const UKF::u_t& u, const double dt);

    // This function implements the observation generation from a state
    UKF::z_t obs_model_f_pos(const UKF::x_t& x);
    UKF::z_t obs_model_f_pose(const UKF::x_t& x);
    UKF::z_t obs_model_f_curv(const UKF::x_t& x);
  }
}
