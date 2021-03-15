#include <mrs_lib/rheiv.h>

namespace ball_filter
{
  namespace rheiv_conic
  {
    constexpr int n_states = 2;
    constexpr int n_params = 6;
    using RHEIV_conic = mrs_lib::RHEIV<n_states, n_params>;

    using xs_t = RHEIV_conic::xs_t;
    using Ps_t = RHEIV_conic::Ps_t;
    using theta_t = RHEIV_conic::theta_t;

    using f_z_t = RHEIV_conic::f_z_t;
    using f_dzdx_t = RHEIV_conic::f_dzdx_t;

    // This function implements the transformation from x to z
    RHEIV_conic::zs_t f_z_f(const RHEIV_conic::xs_t& xs);
    
    // This function implements the jacobian matrix evaluation at x
    RHEIV_conic::dzdx_t f_dzdx_f(const RHEIV_conic::x_t& x);

    // indices of the state interpretations
    // the state represents conic parameters A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
    namespace theta
    {
      enum
      {
        A = 0,
        B,
        C,
        D,
        E,
        F,
      };
    }
  }
}

