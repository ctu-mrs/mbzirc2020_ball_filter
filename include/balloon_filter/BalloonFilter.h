#ifndef BALLOONFILTER_H
#define BALLOONFILTER_H

/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nodelet/nodelet.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// MRS stuff
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/DynamicReconfigureMgr.h>
#include <mrs_lib/geometry_utils.h>

// Boost
#include <boost/circular_buffer.hpp>

// std
#include <string>
#include <mutex>

// local includes
#include <balloon_filter/eight_ukf.h>
#include <balloon_filter/plane_rheiv.h>
/* #include <balloon_filter/conic_rheiv.h> */
#include <balloon_filter/FilterParamsConfig.h>
#include <balloon_filter/ResetEstimates.h>
#include <balloon_filter/Plane.h>
#include <balloon_filter/UKFState.h>
#include <balloon_filter/BallPrediction.h>
#include <balloon_filter/PlaneStamped.h>
#include <object_detect/BallDetections.h>

//}

#define MSG_THROTTLE 0.5

namespace balloon_filter
{
  // shortcut type to the dynamic reconfigure manager template instance
  using drcfg_t = balloon_filter::FilterParamsConfig;
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<drcfg_t>;

  using detections_t = object_detect::BallDetections;
  using ros_poses_t = detections_t::_detections_type;
  using ros_pose_t = ros_poses_t::value_type::_pose_type::_pose_type;
  using ros_cov_t = ros_poses_t::value_type::_pose_type::_covariance_type;

  using RHEIV = rheiv::RHEIV;
  /* using RHEIV_conic = rheiv_conic::RHEIV_conic; */
  using UKF = ukf::UKF;
  using pos_t = RHEIV::x_t;
  using cov_t = RHEIV::P_t;
  using theta_t = RHEIV::theta_t;
  using quat_t = Eigen::Quaterniond;
  struct pos_cov_t
  {
    pos_t pos;
    cov_t cov;
  };

  /* //{ class BalloonFilter */

  class BalloonFilter : public nodelet::Nodelet
  {
    public:
      BalloonFilter() : m_node_name("BalloonFilter") {};
      virtual void onInit();

      bool m_is_initialized;

    private:
      const std::string m_node_name;
      void main_loop([[maybe_unused]] const ros::TimerEvent& evt);
      void rheiv_loop(const ros::TimerEvent& evt);
      void prediction_loop(const ros::TimerEvent& evt);

    private:
      std::unique_ptr<mrs_lib::Profiler> m_profiler_ptr;

    private:

      // --------------------------------------------------------------
      // |                ROS-related member variables                |
      // --------------------------------------------------------------

      /* Parameters, loaded from ROS //{ */
      std::string m_world_frame_id;
      std::string m_uav_frame_id;

      double m_rheiv_fitting_period;
      int m_rheiv_min_pts;
      int m_rheiv_max_pts;
      int m_rheiv_visualization_size;

      ros::Duration m_ukf_init_history_duration;
      double m_gating_distance;
      double m_max_time_since_update;
      double m_min_updates_to_confirm;
      double m_z_bounds_min;
      double m_z_bounds_max;

      double m_prediction_horizon;
      double m_prediction_step;

      double m_ball_speed1;
      double m_ball_speed2;
      ros::Time m_ball_speed_change;

      UKF::x_t m_process_std;
      UKF::x_t m_init_std;

      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      mrs_lib::SubscribeHandlerPtr<detections_t> m_sh_balloons;

      ros::Publisher m_pub_chosen_balloon;
      ros::Publisher m_pub_used_meas;
      ros::Publisher m_pub_ball_prediction;
      ros::Publisher m_pub_pred_path_dbg;
      ros::Publisher m_pub_plane_dbg;
      ros::Publisher m_pub_plane_dbg2;
      ros::Publisher m_pub_used_pts;
      ros::Publisher m_pub_fitted_plane;

      ros::ServiceServer m_reset_estimates_server;

      ros::Timer m_main_loop_timer;
      ros::Timer m_rheiv_loop_timer;
      ros::Timer m_prediction_loop_timer;
      //}

      // | ----------------- RHEIV related variables ---------------- |

      /*  //{ */
      
      RHEIV m_rheiv;
      /* RHEIV_conic m_rheiv_conic; */
      std::mutex m_rheiv_data_mtx;
      boost::circular_buffer<pos_t> m_rheiv_pts;
      boost::circular_buffer<cov_t> m_rheiv_covs;
      boost::circular_buffer<ros::Time> m_rheiv_stamps;
      ros::Time m_rheiv_last_data_update;
      bool m_rheiv_new_data;
      void add_rheiv_data(const pos_t& pos, const cov_t& cov, const ros::Time& stamp)
      {
        std::scoped_lock lck(m_rheiv_data_mtx);
        m_rheiv_pts.push_back(pos);
        m_rheiv_covs.push_back(cov);
        m_rheiv_stamps.push_back(stamp);
        m_rheiv_last_data_update = ros::Time::now();
        m_rheiv_new_data = true;
      };
      std::tuple<boost::circular_buffer<pos_t>, boost::circular_buffer<cov_t>, boost::circular_buffer<ros::Time>, bool, ros::Time> get_rheiv_data()
      {
        std::scoped_lock lck(m_rheiv_data_mtx);
        const bool got_new_data = m_rheiv_new_data;
        m_rheiv_new_data = false;
        return {m_rheiv_pts, m_rheiv_covs, m_rheiv_stamps, got_new_data, m_rheiv_last_data_update};
      };
      
      std::mutex m_rheiv_theta_mtx;
      bool m_rheiv_theta_valid;
      rheiv::theta_t m_rheiv_theta;
      std::tuple<bool, rheiv::theta_t> get_rheiv_status()
      {
        std::scoped_lock lck(m_rheiv_theta_mtx);
        return {m_rheiv_theta_valid, m_rheiv_theta};
      };
      
      //}
      
      // | ------------------ UKF related variables ----------------- |

      /*  //{ */
      
      UKF m_ukf;
      std::mutex m_ukf_estimate_mtx;
      bool m_ukf_estimate_exists;
      UKF::statecov_t m_ukf_estimate;
      ros::Time m_ukf_last_update;
      int m_ukf_n_updates;
      std::tuple<bool, UKF::statecov_t, ros::Time, int> get_ukf_status()
      {
        std::scoped_lock lck(m_ukf_estimate_mtx);
        return {m_ukf_estimate_exists, m_ukf_estimate, m_ukf_last_update, m_ukf_n_updates};
      };
      
      //}

      // | --------------------- Other variables -------------------- |

    private:

      // --------------------------------------------------------------
      // |                helper implementation methods               |
      // --------------------------------------------------------------

      /* get_transform_to_world() method //{ */
      bool get_transform_to_world(const std::string& frame_name, ros::Time stamp, Eigen::Affine3d& tf_out)
      {
        try
        {
          const ros::Duration timeout(1.0 / 100.0);
          geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(m_world_frame_id, frame_name, stamp, timeout);
          tf_out = tf2::transformToEigen(transform.transform);
        } catch (tf2::TransformException& ex)
        {
          ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", frame_name.c_str(), m_world_frame_id.c_str(), ex.what());
          return false;
        }
        return true;
      }
      //}

      cov_t msg2cov(const ros_cov_t& msg_cov);
      cov_t rotate_covariance(const cov_t& covariance, const cov_t& rotation);
      bool point_valid(const pos_t& pt);
      quat_t plane_orientation(const theta_t& plane_theta);
      double plane_angle(const theta_t& plane1, const theta_t& plane2);
      pos_t plane_origin(const theta_t& plane_theta, const pos_t& origin);
      UKF::u_t construct_u(const theta_t& plane_theta, const double speed);
      double ball_speed_at_time(const ros::Time& timestamp);

      /* UKF related methods //{ */
      std::optional<UKF::statecov_t> predict_ukf_estimate(const ros::Time& to_stamp, const theta_t& plane_theta);
      bool update_ukf_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp, pos_cov_t& used_meas, const theta_t& plane_theta);
      UKF::statecov_t estimate_ukf_initial_state(const theta_t& plane_theta);
      bool init_ukf_estimate(const ros::Time& stamp, pos_cov_t& used_meas, const theta_t& plane_theta);
      std::vector<std::pair<UKF::x_t, ros::Time>> predict_states(const UKF::statecov_t initial_statecov, const ros::Time& initial_timestamp, const theta_t& plane_theta, const double prediction_horizon, const double prediction_step);
      //}

      rheiv::theta_t fit_plane(const boost::circular_buffer<pos_t>& points, const boost::circular_buffer<cov_t>& covs);

      pos_t get_pos(const UKF::x_t& x);
      pos_cov_t get_pos_cov(const UKF::statecov_t& statecov);

      geometry_msgs::PoseWithCovarianceStamped to_output_message(const pos_cov_t& estimate, const std_msgs::Header& header);
      visualization_msgs::MarkerArray to_output_message(const theta_t& plane_theta, const std_msgs::Header& header, const pos_t& origin);
      geometry_msgs::PoseStamped to_output_message2(const theta_t& plane_theta, const std_msgs::Header& header, const pos_t& origin);
      nav_msgs::Path to_output_message(const std::vector<std::pair<UKF::x_t, ros::Time>>& predictions, const std_msgs::Header& header, const theta_t& plane_theta);
      sensor_msgs::PointCloud2 to_output_message(const boost::circular_buffer<pos_t>& points, const std_msgs::Header& header);
      balloon_filter::PlaneStamped to_output_message(const theta_t& plane_theta, const std_msgs::Header& header);
      balloon_filter::UKFState to_output_message(const UKF::statecov_t& ukf_statecov);
      balloon_filter::Plane to_output_message(const rheiv::theta_t& plane_theta);

      pos_t get_cur_mav_pos();
      bool find_closest_to(const std::vector<pos_cov_t>& measurements, const pos_t& to_position, pos_cov_t& closest_out, bool use_gating = false);
      bool find_closest(const std::vector<pos_cov_t>& measurements, pos_cov_t& closest_out);

      std::vector<pos_cov_t> message_to_positions(const detections_t& balloon_msg);

      void reset_estimates();
      void reset_ukf_estimate();
      void reset_rheiv_estimate();
      bool reset_estimates_callback([[maybe_unused]] balloon_filter::ResetEstimates::Request& req, balloon_filter::ResetEstimates::Response& resp);
      void load_dynparams(drcfg_t cfg);

  };
  
  //}

}  // namespace balloon_filer

#endif
