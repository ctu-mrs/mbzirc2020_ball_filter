#include <balloon_filter/plane_rheiv.h>

template class mrs_lib::RHEIV<balloon_filter::rheiv::n_states, balloon_filter::rheiv::n_params>;

namespace balloon_filter
{
  namespace rheiv
  {
    using x_t = RHEIV::x_t;
    using u_t = RHEIV::u_t;

    using P_t = RHEIV::P_t;

    using z_t = RHEIV::z_t;
    using zs_t = RHEIV::zs_t;

    using Quat = Eigen::Quaterniond;
    using Vec3 = Eigen::Vector3d;
    using Vec2 = Eigen::Vector2d;

    /* For the plane surface model, there is no need to transform the data. */
    zs_t f_z_f(const xs_t & xs)
    {
      return xs;
    }
  }
}

#ifdef COMPILE_PLANE_RHEIV_TEST

#include <iostream>
#include <fstream>
#include <random>

using namespace balloon_filter;
using namespace balloon_filter::rheiv;

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

double plane_diff(const theta_t& th1, const theta_t& th2)
{
  const auto th1n = th1.normalized();
  const auto th2n = th2.normalized();
  return std::min( (th1n-th2n).norm(), (th1n+th2n).norm() );
}

/* main() function for testing //{ */
int main()
{
  std::string fname = "/home/matous/balloon_workspace/src/ros_packages/balloon_filter/data.csv";
  std::string plane_fname = "/home/matous/balloon_workspace/src/ros_packages/balloon_filter/plane.csv";

  const char delim = ',';
  const Eigen::MatrixXd pts = load_csv<Eigen::MatrixXd>(fname, delim, true);
  const int n_pts = pts.rows();
  [[maybe_unused]] const int cols = pts.cols();
  if (cols < n_states)
  {
    std::cerr << "Wrong input data! At least " << n_states << " columns required, but the data only has " << cols << ", aborting." << std::endl;
    exit(1);
  }
  std::cout << "Loaded " << n_pts << " points" << std::endl;
  const theta_t theta_gt = load_csv<Eigen::MatrixXd>(plane_fname, delim, true).transpose().normalized();

  const f_z_t f_z(f_z_f);
  const dzdx_t dzdx = dzdx_t::Identity();

  RHEIV rheiv(f_z, dzdx, 1e-15, 1e4);

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
    const auto cur_pt = pts.block<1, n_states>(it, 0).transpose();
    const u_t cur_u ( (u_t() << cur_pt, 1).finished() );
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

  std::cout << "norm error (RHEIV):  " << plane_diff(theta_AML, theta_gt) << std::endl;
  std::cout << "norm error (ALS):    " << plane_diff(theta_ALS, theta_gt) << std::endl;
  std::cout << "norm error (ALS, gt):" << plane_diff(theta_ALS_gt, theta_gt) << std::endl;

  std::cout << std::endl;

  std::cout << "avg. error (RHEIV):  " << avg_err_AML << "m" << std::endl;
  std::cout << "avg. error (ALS):    " << avg_err_ALS << "m" << std::endl;
  std::cout << "avg. error (ALS, gt):" << avg_err_ALS_gt << "m" << std::endl;
  std::cout << "avg. error (gt):     " << avg_err_gt << "m" << std::endl;

  /* save_csv(theta, "theta.csv", ',', "a,b,c,d"); */
}
//}

#endif // COMPILE_PLANE_UKF_TEST

