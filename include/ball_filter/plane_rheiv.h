#include <mrs_lib/rheiv.h>

namespace ball_filter
{
  namespace rheiv
  {
    constexpr int n_states = 3;
    constexpr int n_params = 4;
    using RHEIV = mrs_lib::RHEIV<n_states, n_params>;

    using xs_t = RHEIV::xs_t;
    using Ps_t = RHEIV::Ps_t;
    using theta_t = RHEIV::theta_t;

    using f_z_t = RHEIV::f_z_t;
    using dzdx_t = RHEIV::dzdx_t;

    // This function implements the transformation from x to z
    RHEIV::zs_t f_z_f(const RHEIV::xs_t& xs);

    // indices of the state interpretations
    // the state represents plane parameters a*x + b*y + c*z + d = 0
    enum
    {
      x_a = 0,
      x_b,
      x_c,
      x_d,
    };
  }
}
