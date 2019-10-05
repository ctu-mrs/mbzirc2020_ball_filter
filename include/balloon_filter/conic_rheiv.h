#include <mrs_lib/rheiv.h>

namespace balloon_filter
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
    // the state represents conic parameters a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
    enum
    {
      x_a = 0,
      x_b,
      x_c,
      x_d,
      x_e,
      x_f,
    };
  }
}

