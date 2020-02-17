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
#include <mrs_msgs/Float64Stamped.h>

// MRS stuff
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/DynamicReconfigureMgr.h>
#include <mrs_lib/geometry_utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/SafetyZone/SafetyZone.h>
#include <mrs_lib/transformer.h>

// PCL
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_circle.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/registration/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/poisson.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>

// Boost
#include <boost/circular_buffer.hpp>

// std
#include <string>
#include <mutex>

// local includes
#include <balloon_filter/vel_lkf.h>
#include <balloon_filter/eight_ukf.h>
#include <balloon_filter/plane_rheiv.h>
/* #include <balloon_filter/conic_rheiv.h> */
#include <balloon_filter/FilterParamsConfig.h>
#include <balloon_filter/ResetEstimates.h>
#include <balloon_filter/Plane.h>
#include <balloon_filter/UKFState.h>
#include <balloon_filter/BallPrediction.h>
#include <balloon_filter/PlaneStamped.h>
#include <balloon_filter/BallLocation.h>

//}

#define MSG_THROTTLE 0.5

namespace balloon_filter
{
  // shortcut type to the dynamic reconfigure manager template instance
  using drcfg_t = balloon_filter::FilterParamsConfig;
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<drcfg_t>;

  using detection_t = geometry_msgs::PoseWithCovarianceStamped;
  using ros_pose_t = detection_t ::_pose_type::_pose_type;
  using ros_cov_t = detection_t::_pose_type::_covariance_type;

  using RHEIV = rheiv::RHEIV;
  /* using RHEIV_conic = rheiv_conic::RHEIV_conic; */
  using LKF = lkf::LKF;
  using UKF = ukf::UKF;
  using pos_t = RHEIV::x_t;
  using cov_t = RHEIV::P_t;
  using theta_t = RHEIV::theta_t;
  using quat_t = Eigen::Quaterniond;
  using anax_t = Eigen::AngleAxisd;

  struct pos_cov_t
  {
    pos_t pos;
    cov_t cov;
  };
  struct ori_cov_t
  {
    quat_t quat;
    cov_t ypr_cov;
  };
  struct pose_cov_t
  {
    pos_cov_t pos_cov;
    std::optional<ori_cov_t> ori_cov = std::nullopt;
  };
  struct line3d_t
  {
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    double radius; // thickness of the line
  };

  using mrs_lib::get_mutexed;
  using mrs_lib::set_mutexed;
  using mrs_lib::get_set_mutexed;

  /* //{ class BalloonFilter */

  class BalloonFilter : public nodelet::Nodelet
  {
    public:
      BalloonFilter() : m_is_initialized(false), m_safety_area_initialized(false), m_node_name("BalloonFilter") {};
      virtual void onInit();

    private:
      bool m_is_initialized;
      bool m_safety_area_initialized;
      const std::string m_node_name;

      void main_loop(const ros::TimerEvent& evt);
      void rheiv_loop(const ros::TimerEvent& evt);
      void lpf_loop(const ros::TimerEvent& evt);
      void prediction_loop(const ros::TimerEvent& evt);

      void process_measurement(const geometry_msgs::PoseWithCovarianceStamped& msg);
      void init_safety_area(const ros::TimerEvent& evt);

    private:
      std::unique_ptr<mrs_lib::Profiler> m_profiler_ptr;

    private:

      // --------------------------------------------------------------
      // |                ROS-related member variables                |
      // --------------------------------------------------------------

      /* Parameters, loaded from ROS //{ */
      std::string m_world_frame_id;
      std::string m_uav_frame_id;
      std::string m_safety_area_frame;

      double m_rheiv_fitting_period;
      double m_rheiv_max_line_pts_ratio;
      int m_rheiv_min_pts;
      int m_rheiv_max_pts;
      int m_rheiv_visualization_size;

      double m_meas_filt_loglikelihood_threshold;
      double m_meas_filt_covariance_inflation;
      double m_max_time_since_update;
      double m_min_updates_to_confirm;
      double m_bounds_z_min;
      double m_bounds_z_max;

      double m_linefit_threshold_distance;

      double m_circle_min_radius;
      double m_circle_max_radius;
      double m_circle_fit_threshold_distance;
      double m_circle_snap_threshold_distance;

      ros::Duration m_ukf_init_history_duration;
      UKF::x_t m_ukf_process_std;
      UKF::x_t m_ukf_init_std;
      double m_ukf_meas_std_curv_circ;
      double m_ukf_meas_std_curv_line;
      double m_ukf_prediction_horizon;
      double m_ukf_prediction_step;

      bool m_lkf_use_acceleration;
      int m_lkf_n_states;
      int m_lkf_min_init_points;
      ros::Duration m_lkf_init_history_duration;
      LKF::x_t m_lkf_process_std;
      LKF::x_t m_lkf_init_std;
      double m_lkf_max_speed_err;
      double m_lkf_prediction_horizon;
      double m_lkf_prediction_step;

      double m_ball_wire_length;
      
      double m_ball_speed1;
      double m_ball_speed2;
      ros::Time m_ball_speed_change;

      Eigen::MatrixXd m_safety_area_border_points;
      std::vector<Eigen::MatrixXd> m_safety_area_polygon_obstacle_points;
      std::vector<Eigen::MatrixXd> m_safety_area_point_obstacle_points;

      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      /* mrs_lib::SubscribeHandlerPtr<detections_t> m_sh_detections; */
      /* mrs_lib::SubscribeHandlerPtr<detections_t> m_sh_detections_bfx; */
      mrs_lib::SubscribeHandlerPtr<geometry_msgs::PoseWithCovarianceStamped> m_sh_localized;

      ros::Publisher m_pub_dbg;
      ros::Publisher m_pub_pcl_dbg;

      ros::Publisher m_pub_plane_dbg;
      ros::Publisher m_pub_plane_dbg2;
      ros::Publisher m_pub_used_pts;
      ros::Publisher m_pub_fitted_plane;

      ros::Publisher m_pub_chosen_meas;
      ros::Publisher m_pub_chosen_meas_dbg;
      ros::Publisher m_pub_ball_prediction;
      ros::Publisher m_pub_pred_path_dbg;

      ros::Publisher m_pub_circle_dbg;

      ros::ServiceServer m_reset_estimates_server;

      ros::Timer m_main_loop_timer;
      ros::Timer m_rheiv_loop_timer;
      ros::Timer m_prediction_loop_timer;
      ros::Timer m_safety_area_init_timer;
      //}

      mrs_lib::Transformer m_transformer;

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
      bool m_rheiv_fitting;
      void add_rheiv_data(const pos_t& pos, const cov_t& cov, const ros::Time& stamp)
      {
        std::scoped_lock lck(m_rheiv_data_mtx);
        m_rheiv_pts.push_back(pos);
        m_rheiv_covs.push_back(cov);
        m_rheiv_stamps.push_back(stamp);
        m_rheiv_last_data_update = ros::Time::now();
        m_rheiv_new_data = true;
      };
      
      std::mutex m_rheiv_theta_mtx;
      bool m_rheiv_theta_valid;
      rheiv::theta_t m_rheiv_theta;
      
      //}

      // | ------------ Circle fitting related variables ------------ |

      /*  //{ */

      using pt_XYZ_t = pcl::PointXYZ;
      using pc_XYZ_t = pcl::PointCloud<pt_XYZ_t>;
      struct circle3d_t
      {
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
        double radius;
      };
      std::mutex m_circle_mtx;
      bool m_circle_valid;
      circle3d_t m_circle;

      //}

      // | ------------------ UKF related variables ----------------- |

      /*  //{ */
      
      std::mutex m_ukf_mtx;
      UKF m_ukf;
      std::mutex m_ukf_estimate_mtx;
      bool m_ukf_estimate_exists;
      UKF::statecov_t m_ukf_estimate;
      ros::Time m_ukf_last_update;
      int m_ukf_n_updates;
      
      //}

      // | ------------------ LKF related variables ----------------- |

      /*  //{ */
      
      LKF m_lkf;
      std::mutex m_lkf_estimate_mtx;
      bool m_lkf_estimate_exists;
      LKF::statecov_t m_lkf_estimate;
      ros::Time m_lkf_last_update;
      int m_lkf_n_updates;
      
      //}


      // | --------------------- Other variables -------------------- |

      /* ros::Duration m_meas_filt_desired_dt; */
      /* using prev_measurement_t = std::tuple<std::vector<pos_cov_t>, ros::Time>; */
      /* using prev_measurements_t = boost::circular_buffer<prev_measurement_t>; */
      /* prev_measurements_t m_prev_measurements; */
      std::shared_ptr<mrs_lib::SafetyZone> m_safety_area;

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

      template <unsigned num_dimensions>
      double calc_hyp_meas_loglikelihood(const pos_cov_t& hyp, const pos_cov_t& meas, const double cov_inflation);
      cov_t msg2cov(const ros_cov_t& msg_cov, int offset = 0);
      cov_t rotate_covariance(const cov_t& covariance, const cov_t& rotation);
      bool point_valid(const pos_t& pt);
      quat_t plane_orientation(const theta_t& plane_theta);
      double plane_angle(const theta_t& plane1, const theta_t& plane2);
      pos_t plane_origin(const theta_t& plane_theta, const pos_t& origin);
      UKF::u_t construct_u(const theta_t& plane_theta, const double speed);
      double ball_speed_at_time(const ros::Time& timestamp);
      std::tuple<pos_cov_t, double> find_most_likely_association(const pos_cov_t& prev_pt, const std::vector<pos_cov_t>& measurements, const double expected_speed, const double dt, const double cov_inflation);
      std::optional<std::pair<pos_cov_t, pos_cov_t>> find_speed_compliant_measurement(const std::vector<pos_cov_t>& prev_meass, const std::vector<pos_cov_t>& measurements, const double expected_speed, const double dt, const double loglikelihood_threshold, const double cov_inflation);

      /* RHEIV related methods //{ */
      
      rheiv::theta_t fit_plane(const boost::circular_buffer<pos_t>& points, const boost::circular_buffer<cov_t>& covs);
      void reset_rheiv_estimate();
      
      //}

      /* UKF related methods //{ */
      UKF::statecov_t predict_ukf_estimate(const UKF::statecov_t& lkf_estimate, const double dt, const theta_t& plane_theta, const double ball_speed);
      void update_ukf_estimate(const pose_cov_t& measurement, const ros::Time& stamp, const theta_t& plane_theta);
      void update_ukf_estimate(const circle3d_t& circle, const theta_t& rheiv_plane);
      std::optional<UKF::statecov_t> estimate_ukf_initial_state(const theta_t& plane_theta);
      void init_ukf_estimate(const ros::Time& stamp, const theta_t& plane_theta);
      std::vector<std::pair<UKF::x_t, ros::Time>> predict_ukf_states(const UKF::statecov_t initial_statecov, const ros::Time& initial_timestamp, const theta_t& plane_theta, const double prediction_horizon, const double prediction_step);
      void reset_ukf_estimate();
      //}

      /* LKF related methods //{ */
      LKF::statecov_t predict_lkf_estimate(const LKF::statecov_t& lkf_estimate, const double dt);
      void update_lkf_estimate(const pose_cov_t& measurement, const ros::Time& stamp);
      std::optional<LKF::statecov_t> estimate_lkf_initial_state();
      void init_lkf_estimate(const ros::Time& stamp);
      std::vector<std::pair<LKF::x_t, ros::Time>> predict_lkf_states(const LKF::statecov_t initial_statecov, const ros::Time& initial_timestamp, const double prediction_horizon, const double prediction_step, const double set_speed);
      void reset_lkf_estimate();
      //}

      /* Circle fitting related methods //{ */
      
      void reset_circle_estimate();
      
      //}

      template <class T>
      pos_t get_pos(const T& x);
      template <class T>
      pos_cov_t get_pos_cov(const T& statecov);

      balloon_filter::BallLocation to_output_message(const pos_cov_t& estimate, const std_msgs::Header& header);
      geometry_msgs::PoseWithCovarianceStamped to_output_message2(const pos_cov_t& estimate, const std_msgs::Header& header);
      visualization_msgs::MarkerArray to_output_message(const theta_t& plane_theta, const std_msgs::Header& header, const pos_t& origin);
      geometry_msgs::PoseStamped to_output_message2(const theta_t& plane_theta, const std_msgs::Header& header, const pos_t& origin);
      nav_msgs::Path to_output_message(const std::vector<std::pair<UKF::x_t, ros::Time>>& predictions, const std_msgs::Header& header, const theta_t& plane_theta);
      nav_msgs::Path to_output_message(const std::vector<std::pair<LKF::x_t, ros::Time>>& predictions, const std_msgs::Header& header);
      sensor_msgs::PointCloud2 to_output_message(const boost::circular_buffer<pos_t>& points, const std_msgs::Header& header);
      balloon_filter::PlaneStamped to_output_message(const theta_t& plane_theta, const std_msgs::Header& header);
      balloon_filter::LKFState to_output_message(const LKF::statecov_t& lkf_statecov);
      balloon_filter::UKFState to_output_message(const UKF::statecov_t& ukf_statecov);
      balloon_filter::Plane to_output_message(const rheiv::theta_t& plane_theta);
      visualization_msgs::MarkerArray line_visualization(const line3d_t& line, const std_msgs::Header& header, const std::string& text);
      visualization_msgs::MarkerArray circle_visualization(const circle3d_t& circle, const std_msgs::Header& header);

      pos_t get_cur_mav_pos();

      void reset_estimates();
      bool reset_estimates_callback([[maybe_unused]] balloon_filter::ResetEstimates::Request& req, balloon_filter::ResetEstimates::Response& resp);
      void load_dynparams(drcfg_t cfg);

  };
  
  //}

}  // namespace balloon_filer

#endif
