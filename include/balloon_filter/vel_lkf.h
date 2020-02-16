#include <mrs_lib/lkf.h>

namespace balloon_filter
{
  namespace lkf
  {
    // indices of the state interpretations
    namespace x
    {
      enum
      {
        x = 0,  // 3D x-coordinate of the ball position
        y,      // 3D y-coordinate of the ball position
        z,      // 3D z-coordinate of the ball position
        dx,     // 3D x-coordinate of the ball speed
        dy,     // 3D y-coordinate of the ball speed
        dz,     // 3D z-coordinate of the ball speed
        ddx,    // 3D x-coordinate of the ball acceleration
        ddy,    // 3D y-coordinate of the ball acceleration
        ddz,    // 3D z-coordinate of the ball acceleration
        LENGTH
      };
    }

    // indices of the input interpretations
    namespace u
    {
      enum
      {
        LENGTH
      };
    }

    namespace z
    {
      // indices of the measurement interpretations
      enum
      {
        x = 0, // 3D x-coordinate of the ball position
        y,     // 3D y-coordinate of the ball position
        z,     // 3D z-coordinate of the ball position
        dx,     // 3D x-coordinate of the ball speed
        dy,     // 3D y-coordinate of the ball speed
        dz,     // 3D z-coordinate of the ball speed
        LENGTH
      };
    }

    constexpr int n_states = -1;
    constexpr int n_inputs = u::LENGTH;
    constexpr int n_measurements = -1;
    using LKF = mrs_lib::LKF<n_states, n_inputs, n_measurements>;
  }
}

