#include <balloon_filter/eight_ukf.h>

template class mrs_lib::UKF<balloon_filter::ukf::n_states, balloon_filter::ukf::n_inputs, balloon_filter::ukf::n_measurements>;

namespace balloon_filter
{
  namespace ukf
  {
    using x_t = UKF::x_t;
    using P_t = UKF::P_t;
    using u_t = UKF::u_t;
    using z_t = UKF::z_t;
    using Q_t = UKF::Q_t;
    using R_t = UKF::R_t;
    using statecov_t = UKF::statecov_t;
    using tra_model_t = UKF::transition_model_t;
    using obs_model_t = UKF::observation_model_t;

    using Quat = Eigen::Quaterniond;
    using Vec3 = Eigen::Vector3d;
    using Vec2 = Eigen::Vector2d;

    // from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
    template <typename T>
    T sign(T val)
    {
      return (T(0) < val) - (val < T(0));
    }

    UKF::x_t tra_model_f(const UKF::x_t& in, const UKF::u_t& u, const double dt)
    {
      x_t out;

      // Calculate the complete current state with redundant variables
      const double yaw = in(x::yaw);
      const double speed = std::max(u(u::s), 0.0);
      const double curv = in(x::c);
      // reference for curvature: https://web.ma.utexas.edu/users/m408m/Display13-4-3.shtml
      const Quat quat = Quat(u(u::qw), u(u::qx), u(u::qy), u(u::qz)).normalized();
      const Vec3 tang(cos(yaw), sin(yaw), 0.0);
      const Vec3 norm(cos(yaw + M_PI_2), sin(yaw + M_PI_2), 0.0);
      const Vec3 vel_tang = speed*tang;
      const Vec3 acc_norm = curv*speed*speed*norm;
      const Vec3 dpos = vel_tang*dt + 0.5*acc_norm*dt*dt;
      const Vec3 pos_world(in(x::x), in(x::y), in(x::z));

      // Calculate the next estimated state
      const Vec3 n_pos_world = pos_world + quat*dpos;
      /* const double n_speed = speed; // assume constant speed */
      const double n_yaw = yaw + speed*curv*dt;
      const double n_curv = curv;

      // Copy the calculated values to the respective states
      out(x::x) = n_pos_world.x();
      out(x::y) = n_pos_world.y();
      out(x::z) = n_pos_world.z();
      out(x::yaw) = n_yaw;
      /* out(x::s) = n_speed; */
      out(x::c) = n_curv;

      return out;
    }

    // This function implements the observation generation from a state
    UKF::z_t obs_model_f(const UKF::x_t& state)
    {
      z_t observation;
      observation(z::x) = state(x::x);
      observation(z::y) = state(x::y);
      observation(z::z) = state(x::z);
      return observation;
    }

    double normalize_angle(double in)
    {
      double out = std::fmod(in, 2*M_PI);
      if (out > M_PI)
        out -= 2*M_PI;
      else if (out < -M_PI)
        out += 2*M_PI;
      return out;
    }

    UKF::x_t normalize_state(const UKF::x_t& in)
    {
      UKF::x_t out = in;
      out(x::yaw) = normalize_angle(in(x::yaw));
      return out;
    }
  }
}

#ifdef COMPILE_EIGHT_UKF_TEST

#include <iostream>
#include <fstream>
#include <random>

using namespace balloon_filter;
using namespace balloon_filter::ukf;

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

/* normal_randmat() function //{ */
template <int rows>
Eigen::Matrix<double, rows, 1> normal_randmat(const Eigen::Matrix<double, rows, rows>& cov)
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

/* main() function for testing //{ */
int main()
{
  std::string fname = "data.csv";

  const char delim = ',';
  const Eigen::MatrixXd pts = load_csv<Eigen::MatrixXd>(fname, delim, true);
  const int n_pts = pts.rows();
  const int cols = pts.cols();
  std::cout << "Loaded " << n_pts << " points" << std::endl;

  const double dt = 1.0;
  Q_t Q = Q_t::Identity();
  Q.block<3, 3>(0, 0) *= 1e0;
  Q(3, 3) *= 1e-2;
  Q(4, 4) *= 1e-1;
  Q(5, 5) *= 5e-1;
  Q.block<4, 4>(6, 6) *= 1e-6;
  tra_model_t tra_model(tra_model_f);
  obs_model_t obs_model(obs_model_f);

  UKF ukf(tra_model, obs_model);

  const double R_std = 1e-1;
  const R_t R = R_std*R_t::Identity();
  const x_t x0((x_t() << 
      0,
      0,
      0,
      0.5,
      0.5,
      0,
      1,
      0,
      0,
      1
      ).finished()
      );
  const P_t P0 = 1e2*Q;
  double err_acc = 0.0;
  const int err_states = std::min(cols, n_states);
  statecov_t sc {x0, P0};
  Eigen::MatrixXd est_states(n_pts, n_states);
  for (int it = 0; it < n_pts; it++)
  {
    std::cout << "iteration " << it << std::endl;
    const auto cur_gt = pts.block(it, 0, 1, err_states).transpose();
    const auto cur_pt = pts.block<1, 3>(it, 0).transpose();
    std::cout << "state ground-truth: " << std::endl << cur_gt.transpose() << std::endl;
    /* const auto cur_meas = cur_pt; */
    const auto cur_meas = cur_pt + normal_randmat(R);

    std::cout << "ukf state: " << std::endl << sc.x.transpose() << std::endl;
    sc = ukf.predict(sc, u_t(), Q, dt);
    sc.x = normalize_state(sc.x);
    std::cout << "state prediction: " << std::endl << sc.x.transpose() << std::endl;
    sc = ukf.correct(sc, cur_meas, R);
    sc.x = normalize_state(sc.x);
    const auto cur_est = sc.x.block(0, 0, err_states, 1);
    const double cur_err = (cur_gt-cur_est).norm();
    err_acc += cur_err;
    std::cout << "state correction: " << std::endl << sc.x.transpose() << std::endl;
    est_states.block<1, n_states>(it, 0) = sc.x.transpose();

    /* std::cout << "state covariance: " << std::endl << sc.P << std::endl; */
    std::cout << "current error: " << std::endl << cur_err << std::endl;
    std::cout << "accumulated error: " << std::endl << err_acc << std::endl;
  }

  save_csv(est_states, "est.csv", ',', "x,y,z,yaw,speed,curvature,qw,qx,qy,qz");
}
//}

#endif // COMPILE_EIGHT_UKF_TEST
