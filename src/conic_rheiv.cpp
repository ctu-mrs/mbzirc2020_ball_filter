#include <balloon_filter/conic_rheiv.h>

template class mrs_lib::RHEIV<balloon_filter::rheiv_conic::n_states, balloon_filter::rheiv_conic::n_params>;

namespace balloon_filter
{
  namespace rheiv_conic
  {
    using x_t = RHEIV_conic::x_t;
    using u_t = RHEIV_conic::u_t;

    using P_t = RHEIV_conic::P_t;

    using z_t = RHEIV_conic::z_t;
    using zs_t = RHEIV_conic::zs_t;
    using dzdx_t = RHEIV_conic::dzdx_t;

    using Quat = Eigen::Quaterniond;
    using Vec3 = Eigen::Vector3d;
    using Vec2 = Eigen::Vector2d;

    zs_t f_z_f(const xs_t& xs)
    {
      const int n = xs.cols();
      zs_t ret = zs_t::Zero(zs_t::RowsAtCompileTime, n);
      const auto& x_vals = xs.block(0, 0, 1, n).array();
      const auto& y_vals = xs.block(1, 0, 1, n).array();
      ret.block(0, 0, 1, n) = x_vals*x_vals; // x^2
      ret.block(1, 0, 1, n) = x_vals*y_vals; // x*y
      ret.block(2, 0, 1, n) = y_vals*y_vals; // y^2
      ret.block(3, 0, 1, n) = x_vals;        // x
      ret.block(4, 0, 1, n) = y_vals;        // y
      return ret;
    }

    dzdx_t f_dzdx_f(const x_t& x)
    {
      const dzdx_t ret((dzdx_t() << 
            2*x(0), 0,
            x(1), x(0),
            0, 2*x(1),
            1, 0,
            0, 1).finished());
      return ret;
    }
  }
}

#ifdef COMPILE_CONIC_RHEIV_TEST

#include <iostream>
#include <fstream>
#include <random>

using namespace balloon_filter;
using namespace balloon_filter::rheiv_conic;

/* load_csv() function //{ */
// from https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
template<typename M>
M load_csv(const std::string& path, const char delim = ',', bool skip_header = false)
{
  std::ifstream indata(path);
  if (!indata.is_open())
  {
    std::cerr << "Could not open " << path << std::endl;
    return M();
  }
  std::string line;
  std::vector<double> values;
  int rows = 0;
  while (std::getline(indata, line))
  {
    if (skip_header)
    {
      skip_header = false;
      continue;
    }
    std::stringstream line_stream(line);
    std::string cell;
    while (std::getline(line_stream, cell, delim))
    {
      values.push_back(std::stod(cell));
    }
    rows++;
  }
  return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}
//}

/* save_csv() function //{ */
template<typename M>
void save_csv(const M& mat, const std::string& path, const char delim = ',', const std::string header = std::string())
{
  std::ofstream odata(path);
  if (!odata.is_open())
  {
    std::cerr << "Could not open " << path << std::endl;
    return;
  }
  if (!header.empty())
    odata << header << std::endl;
  for (int row = 0; row < mat.rows(); row++)
  {
    for (int col = 0; col < mat.cols(); col++)
    {
      odata << mat(row, col);
      if (col != mat.cols() -1)
        odata << delim;
    }
    odata << std::endl;
  }
}
//}

/* normal_randvec() function //{ */
template <int rows>
Eigen::Matrix<double, rows, 1> normal_randvec(const Eigen::Matrix<double, rows, rows>& cov)
{
    static std::random_device rd{};
    static std::mt19937 gen{rd()};
    static std::normal_distribution<> d{0,1};
    Eigen::Matrix<double, rows, 1> ret;
    for (int row = 0; row < rows; row++)
      ret(row, 0) = d(gen);
    return cov*ret;
}
//}

double theta_diff(const theta_t& th1, const theta_t& th2)
{
  const auto th1n = th1.normalized();
  const auto th2n = th2.normalized();
  return std::min( (th1n-th2n).norm(), (th1n+th2n).norm() );
}

/* main() function for testing //{ */
int main()
{
  std::string fname = "/home/matous/balloon_workspace/src/ros_packages/balloon_filter/data/conic_data.csv";
  std::string conic_fname = "/home/matous/balloon_workspace/src/ros_packages/balloon_filter/data/conic.csv";

  const char delim = ',';
  const Eigen::MatrixXd pts = load_csv<Eigen::MatrixXd>(fname, delim, false);
  const int n_pts = pts.rows();
  [[maybe_unused]] const int cols = pts.cols();
  if (cols < n_states)
  {
    std::cerr << "Wrong input data! At least " << n_states << " columns required, but the data only has " << cols << ", aborting." << std::endl;
    exit(1);
  }
  std::cout << "Loaded " << n_pts << " points" << std::endl;
  const theta_t theta_gt = load_csv<Eigen::MatrixXd>(conic_fname, delim, false).transpose().normalized();

  const f_z_t f_z(f_z_f);
  const f_dzdx_t f_dzdx(f_dzdx_f);

  RHEIV_conic rheiv(f_z, f_dzdx, 1e-15, 1e4);

  xs_t gts = pts.transpose().block(0, 0, n_states, n_pts);
  xs_t xs(n_states, n_pts);
  Ps_t Ps(n_pts);
  for (int it = 0; it < n_pts; it++)
  {
    const auto cur_pt = pts.block<1, n_states>(it, 0).transpose();
    const P_t tmp = P_t::Random();
    const P_t P = 5*tmp*tmp.transpose();
    const x_t cur_x = cur_pt + normal_randvec(P);
    xs.block(0, it, n_states, 1) = cur_x;
    Ps.at(it) = 0.1*P;
  }

  const auto theta_AML = rheiv.fit(xs, Ps);
  const auto theta_ALS = rheiv.fit_ALS(xs);
  const auto theta_ALS_gt = rheiv.fit_ALS(gts);

  double avg_err_AML = 0.0;
  double avg_err_ALS = 0.0;
  double avg_err_ALS_gt = 0.0;
  double avg_err_gt = 0.0;
  for (int it = 0; it < n_pts; it++)
  {
    const x_t cur_x = pts.block<1, n_states>(it, 0).transpose();
    const z_t cur_z = f_z_f(cur_x);
    const u_t cur_u ( (u_t() << cur_z, 1).finished() );
    const double err_AML = abs(theta_AML.transpose() * cur_u);
    const double err_ALS = abs(theta_ALS.transpose() * cur_u);
    const double err_ALS_gt = abs(theta_ALS_gt.transpose() * cur_u);
    const double err_gt = abs(theta_gt.transpose() * cur_u);
    avg_err_AML += err_AML;
    avg_err_ALS += err_ALS;
    avg_err_ALS_gt += err_ALS_gt;
    avg_err_gt += err_gt;
  }
  avg_err_AML /= n_pts;
  avg_err_ALS /= n_pts;
  avg_err_ALS_gt /= n_pts;
  avg_err_gt /= n_pts;

  std::cout << "theta (RHEIV):  " << std::endl << theta_AML << std::endl;
  std::cout << "theta (ALS):    " << std::endl << theta_ALS << std::endl;
  std::cout << "theta (ALS, gt):" << std::endl << theta_ALS_gt << std::endl;
  std::cout << "theta (gt):     " << std::endl << theta_gt << std::endl;

  std::cout << std::endl;

  std::cout << "norm error (RHEIV):  " << theta_diff(theta_AML, theta_gt) << std::endl;
  std::cout << "norm error (ALS):    " << theta_diff(theta_ALS, theta_gt) << std::endl;
  std::cout << "norm error (ALS, gt):" << theta_diff(theta_ALS_gt, theta_gt) << std::endl;

  std::cout << std::endl;

  std::cout << "avg. error (RHEIV):  " << avg_err_AML << "m" << std::endl;
  std::cout << "avg. error (ALS):    " << avg_err_ALS << "m" << std::endl;
  std::cout << "avg. error (ALS, gt):" << avg_err_ALS_gt << "m" << std::endl;
  std::cout << "avg. error (gt):     " << avg_err_gt << "m" << std::endl;

  /* save_csv(theta, "theta.csv", ',', "a,b,c,d"); */
}
//}

#endif // COMPILE_CONIC_RHEIV_TEST


