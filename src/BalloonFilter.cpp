#include <balloon_filter/BalloonFilter.h>

namespace balloon_filter
{

  /* main_loop() method //{ */
  void BalloonFilter::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    load_dynparams(m_drmgr_ptr->config);

    if (m_sh_balloons->new_data())
    {
      const auto balloons = *(m_sh_balloons->get_data());

      if (!balloons.poses.empty())
      {
        /* ROS_INFO("[%s]: Processing %lu new detections vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv", m_node_name.c_str(), balloons.poses.size()); */

        // transform the message to usable measurement format
        std::vector<pos_cov_t> measurements = message_to_positions(balloons);

        for (const auto& meas : measurements)
          add_rheiv_data(meas.pos, meas.cov);

        // copy the latest plane fit
        const auto [plane_theta_valid, plane_theta] = get_rheiv_status();

        if (plane_theta_valid && (m_pub_plane_dbg.getNumSubscribers() > 0 || m_pub_plane_dbg2.getNumSubscribers()))
        {
          std_msgs::Header header;
          header.frame_id = m_world_frame;
          header.stamp = ros::Time::now();
          if (m_pub_plane_dbg.getNumSubscribers())
            m_pub_plane_dbg.publish(to_output_message(plane_theta, header, get_pos(m_ukf_estimate.x)));
          if (m_pub_plane_dbg2.getNumSubscribers())
            m_pub_plane_dbg2.publish(to_output_message2(plane_theta, header, get_pos(m_ukf_estimate.x)));
        }

        // check if we have all data that is needed
        if (!measurements.empty() && plane_theta_valid)
        {
          // if all data is available, try to either initialize or update the UKF
          pos_cov_t used_meas;
          bool used_meas_valid = false;
          if (m_ukf_estimate_exists)
            used_meas_valid = update_ukf_estimate(measurements, balloons.header.stamp, used_meas, plane_theta);
          else
            used_meas_valid = init_ukf_estimate(measurements, balloons.header.stamp, used_meas);

          if (used_meas_valid)
          {
            /* publish the used measurement for debugging and visualisation purposes //{ */
            std_msgs::Header header;
            header.frame_id = m_world_frame;
            header.stamp = balloons.header.stamp;
            m_pub_used_meas.publish(to_output_message(used_meas, header));
            //}
          }
        } else
        {
          if (!plane_theta_valid)
          {
            ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[UKF] RHEIV plane theta estimate unavailable, cannot update UKF!");
          }
          if (!measurements.empty())
          {
            ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[UKF] Got empty detection message, cannot update UKF!");
          }
        }  // if (!measurements.empty() && plane_theta_valid)

        ros::Duration del = ros::Time::now() - balloons.header.stamp;
        ROS_INFO_STREAM_THROTTLE(MSG_THROTTLE, "[UKF]: delay (from image acquisition): " << del.toSec() * 1000.0 << "ms");
        /* ROS_INFO("[%s]: New data processed          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^", m_node_name.c_str()); */
      } else
      {
        ROS_INFO_THROTTLE(MSG_THROTTLE, "[%s]: Empty detections message received", m_node_name.c_str());
      }
    }
    if ((ros::Time::now() - m_ukf_last_update).toSec() >= m_max_time_since_update)
    {
      reset_ukf_estimate();
    }

    if (m_ukf_estimate_exists && m_ukf_n_updates > m_min_updates_to_confirm)
    {
      const auto [plane_theta_valid, plane_theta] = get_rheiv_status();
      if (plane_theta_valid)
      {
        std_msgs::Header header;
        header.frame_id = m_world_frame;
        header.stamp = ros::Time::now();
        auto cur_estimate = predict_ukf_estimate(header.stamp, plane_theta);
        Eigen::IOFormat short_fmt(3);
        cur_estimate.x.format(short_fmt);
        const auto cur_pos_cov = get_pos_cov(cur_estimate);
        m_pub_chosen_balloon.publish(to_output_message(cur_pos_cov, header));
        ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[UKF]: Current UKF prediction:" << std::endl
                                                                                << "[\tx\t\ty\t\tz\t\tyaw\t\tspd\t\tcur\t]" << std::endl
                                                                                << "[" << cur_estimate.x.transpose() << "]");
      }
    }
  }
  //}

  /* rheiv_loop() method //{ */
  void BalloonFilter::rheiv_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    // threadsafe copy the data to be fitted with the plane
    const auto [rheiv_pts, rheiv_covs] = get_rheiv_data();

    ros::Time stamp = ros::Time::now();
    bool success = false;
    rheiv::theta_t theta;
    if (rheiv_pts.size() > (size_t)m_rheiv_min_pts)
    {
      ros::Time fit_time_start = ros::Time::now();
      // Fitting might throw an exception, so we better try/catch it!
      try
      {
        theta = fit_plane(rheiv_pts, rheiv_covs);
        if (abs(plane_angle(-theta, m_rheiv_theta)) < abs(plane_angle(theta, m_rheiv_theta)))
          theta = -theta;
        double angle_diff = plane_angle(theta, m_rheiv_theta);

        // If everything went well, save the results and print a nice message
        success = true;
        stamp = ros::Time::now();
        {
          std::scoped_lock lck(m_rheiv_theta_mtx);
          m_rheiv_theta_valid = true;
          m_rheiv_theta = theta;
        }
        ROS_INFO_STREAM_THROTTLE(MSG_THROTTLE, "[RHEIV]: Fitted new plane estimate to " << rheiv_pts.size() << " points in " << (stamp - fit_time_start).toSec()
                                                                                        << "s (angle diff: " << angle_diff << "):" << std::endl
                                                                                        << "[" << theta.transpose() << "]");
      }
      catch (const mrs_lib::eigenvector_exception& ex)
      {
        // Fitting threw exception, notify the user.
        stamp = ros::Time::now();
        ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[RHEIV]: Could not fit plane: '" << ex.what() << "' (took " << (stamp - fit_time_start).toSec() << "s).");
      }
    } else
    {
      // Still waiting for enough points to fit the plane through.
      ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[RHEIV]: Not enough points to fit plane (" << rheiv_pts.size() << "/ " << m_rheiv_min_pts << ").");
    }

    if (m_pub_used_pts.getNumSubscribers() > 0)
    {
      std_msgs::Header header;
      header.frame_id = m_world_frame;
      header.stamp = ros::Time::now();
      m_pub_used_pts.publish(to_output_message(rheiv_pts, header));
    }

    if (success)
    {
      std_msgs::Header header;
      header.frame_id = m_world_frame;
      header.stamp = stamp;
      m_pub_fitted_plane.publish(to_output_message(theta, header));
    }
    /* // start the fitting process again after the desired delay */
    /* ros::Duration d_remaining = ros::Duration(m_rheiv_fitting_period) - d_cbk; */
    /* if (d_remaining < ros::Duration(0)) */
    /*   d_remaining = ros::Duration(0); */
    /* ROS_WARN_STREAM("[RHEIV]: Next fit in " << d_remaining.toSec() << "s."); */
    /* m_rheiv_loop_timer.stop(); */
    /* m_rheiv_loop_timer.setPeriod(d_remaining); */
    /* m_rheiv_loop_timer.start(); */
  }
  //}

  /* prediction_loop() method //{ */
  void BalloonFilter::prediction_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    const auto [ukf_estimate_exists, ukf_estimate, ukf_last_update, ukf_n_updates] = get_ukf_status();
    const auto [plane_theta_valid, plane_theta] = get_rheiv_status();
    if (ukf_estimate_exists && ukf_n_updates > m_min_updates_to_confirm && plane_theta_valid)
    {
      const auto predictions = predict_states(ukf_estimate, ukf_last_update, plane_theta, m_prediction_horizon, m_prediction_step);
      balloon_filter::BallPrediction message;
      message.header.frame_id = m_world_frame;
      message.header.stamp = ukf_last_update;

      balloon_filter::Plane fitted_plane = to_output_message(plane_theta);
      balloon_filter::UKFState ukf_state = to_output_message(ukf_estimate);
      balloon_filter::FilterState filter_state;
      nav_msgs::Path predicted_path = to_output_message(predictions, message.header, plane_theta);

      filter_state.fitted_plane = fitted_plane;
      filter_state.ukf_state = ukf_state;
      message.filter_state = filter_state;
      message.predicted_path = predicted_path;

      m_pub_pred_path_dbg.publish(predicted_path);
      m_pub_ball_prediction.publish(message);
    }
  }
  //}

  // --------------------------------------------------------------
  // |                     UKF related methods                    |
  // --------------------------------------------------------------

  /* predict_ukf_estimate() method //{ */
  UKF::statecov_t BalloonFilter::predict_ukf_estimate(const ros::Time& to_stamp, const theta_t& plane_theta)
  {
    const double dt = (to_stamp - m_ukf_last_update).toSec();
    const UKF::Q_t Q = dt * m_process_std.asDiagonal();

    const UKF::u_t u = plane_theta_to_ukf_u(plane_theta);
    auto ukf_estimate = m_ukf_estimate;

    const auto ret = m_ukf.predict(ukf_estimate, u, Q, dt);
    return ret;
  }
  //}

  /* update_ukf_estimate() method //{ */
  bool BalloonFilter::update_ukf_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp, pos_cov_t& used_meas, const theta_t& plane_theta)
  {
    auto [ukf_estimate_exists, ukf_estimate, ukf_last_update, ukf_n_updates] = get_ukf_status();

    pos_cov_t closest_meas;
    const auto cur_pos = get_pos(ukf_estimate.x);
    bool meas_valid = find_closest_to(measurements, cur_pos, closest_meas, true);

    if (meas_valid)
    {
      ROS_INFO_THROTTLE(MSG_THROTTLE, "[UKF]: Updating current estimate using point [%.2f, %.2f, %.2f]", closest_meas.pos.x(), closest_meas.pos.y(),
                        closest_meas.pos.z());
      const double dt = (stamp - ukf_last_update).toSec();
      const UKF::u_t u = plane_theta_to_ukf_u(plane_theta);
      const UKF::Q_t Q = dt * m_process_std.asDiagonal();

      ukf_estimate = m_ukf.predict(ukf_estimate, u, Q, dt);
      ukf_estimate = m_ukf.correct(ukf_estimate, closest_meas.pos, closest_meas.cov);
      ukf_last_update = stamp;
      ukf_n_updates++;
      used_meas = closest_meas;
      ukf_estimate_exists = true;

      {
        std::scoped_lock lck(m_ukf_estimate_mtx);
        m_ukf_estimate_exists = ukf_estimate_exists;
        m_ukf_estimate = ukf_estimate;
        m_ukf_last_update = ukf_last_update;
        m_ukf_n_updates = ukf_n_updates;
      }
    } else
    {
      ROS_INFO("[UKF]: No point is close enough to [%.2f, %.2f, %.2f]", ukf_estimate.x.x(), ukf_estimate.x.y(), ukf_estimate.x.z());
    }
    return meas_valid;
  }
  //}

  /* init_ukf_estimate() method //{ */
  bool BalloonFilter::init_ukf_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp, pos_cov_t& used_meas)
  {
    pos_cov_t closest_meas;
    bool meas_valid = find_closest(measurements, closest_meas);
    if (meas_valid)
    {
      ROS_INFO("[UKF]: Initializing estimate using point [%.2f, %.2f, %.2f]", closest_meas.pos.x(), closest_meas.pos.y(), closest_meas.pos.z());

      {
        std::scoped_lock lck(m_ukf_estimate_mtx);

        m_ukf_estimate.x = UKF::x_t::Zero();
        m_ukf_estimate.x.block<3, 1>(0, 0) = closest_meas.pos;

        m_ukf_estimate.P = m_init_std.asDiagonal();
        m_ukf_estimate.P.block<3, 3>(0, 0) = closest_meas.cov;
        m_ukf_estimate_exists = true;
        m_ukf_last_update = stamp;
        m_ukf_n_updates = 1;
      }
      used_meas = closest_meas;
    } else
    {
      ROS_INFO("[UKF]: No point is valid for estimate initialization");
    }
    return meas_valid;
  }
  //}

  /* reset_ukf_estimate() method //{ */
  void BalloonFilter::reset_ukf_estimate()
  {
    m_ukf_estimate_exists = false;
    m_ukf_last_update = ros::Time::now();
    m_ukf_n_updates = 0;
    ROS_INFO("[%s]: Current chosen balloon ==RESET==.", m_node_name.c_str());
  }
  //}

  /* predict_states() method //{ */
  std::vector<std::pair<UKF::x_t, ros::Time>> BalloonFilter::predict_states(const UKF::statecov_t initial_statecov, const ros::Time& initial_timestamp,
                                                                            const theta_t& plane_theta, const double prediction_horizon,
                                                                            const double prediction_step)
  {
    assert(prediction_step > 0.0);
    assert(prediction_horizon > 0.0);
    const int n_pts = round(prediction_horizon / prediction_step);
    const UKF::u_t u = plane_theta_to_ukf_u(plane_theta);
    const UKF::Q_t Q = prediction_step * m_process_std.asDiagonal();

    UKF::statecov_t statecov = initial_statecov;
    ros::Time timestamp = initial_timestamp;
    std::vector<std::pair<UKF::x_t, ros::Time>> ret;
    ret.reserve(n_pts);
    ret.push_back({statecov.x, timestamp});
    for (int it = 0; it < n_pts; it++)
    {
      statecov = m_ukf.predict(statecov, u, Q, prediction_step);
      timestamp += ros::Duration(prediction_step);
      ret.push_back({statecov.x, timestamp});
    }
    return ret;
  }
  //}

  // --------------------------------------------------------------
  // |                    RHEIV related methods                   |
  // --------------------------------------------------------------

  /* fit_plane() method //{ */
  rheiv::theta_t BalloonFilter::fit_plane(const boost::circular_buffer<pos_t>& points, const boost::circular_buffer<cov_t>& covs)
  {
    const rheiv::theta_t ret = m_rheiv.fit(points.begin(), points.end(), covs.begin(), covs.end());
    return ret;
  }
  //}

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  /* get_pos() method //{ */
  pos_t BalloonFilter::get_pos(const UKF::x_t& x)
  {
    return x.block<3, 1>(0, 0);
  }
  //}

  /* get_pos_cov() method //{ */
  pos_cov_t BalloonFilter::get_pos_cov(const UKF::statecov_t& statecov)
  {
    pos_cov_t ret;
    ret.pos = get_pos(statecov.x);
    ret.cov = statecov.P.block<3, 3>(0, 0);
    return ret;
  }
  //}

  /* to_output_message() method overloads //{ */
  /* geometry_msgs::PoseWithCovarianceStamped //{ */
  geometry_msgs::PoseWithCovarianceStamped BalloonFilter::to_output_message(const pos_cov_t& estimate, const std_msgs::Header& header)
  {
    geometry_msgs::PoseWithCovarianceStamped ret;

    ret.header = header;
    ret.pose.pose.position.x = estimate.pos.x();
    ret.pose.pose.position.y = estimate.pos.y();
    ret.pose.pose.position.z = estimate.pos.z();
    ret.pose.pose.orientation.x = 0.0;
    ret.pose.pose.orientation.y = 0.0;
    ret.pose.pose.orientation.z = 0.0;
    ret.pose.pose.orientation.w = 1.0;

    for (int r = 0; r < 6; r++)
    {
      for (int c = 0; c < 6; c++)
      {
        if (r < 3 && c < 3)
          ret.pose.covariance[r * 6 + c] = estimate.cov(r, c);
        else if (r == c)
          ret.pose.covariance[r * 6 + c] = 666;
        else
          ret.pose.covariance[r * 6 + c] = 0.0;
      }
    }

    return ret;
  }
  //}

  /* visualization_msgs::MarkerArray //{ */
  visualization_msgs::MarkerArray BalloonFilter::to_output_message(const theta_t& plane_theta, const std_msgs::Header& header, const pos_t& origin)
  {
    visualization_msgs::MarkerArray ret;

    const auto pos = plane_origin(plane_theta, origin);
    const auto quat = plane_orientation(plane_theta);

    const double size = m_rheiv_visualization_size;
    geometry_msgs::Point ptA;
    ptA.x = size;
    ptA.y = size;
    ptA.z = 0;
    geometry_msgs::Point ptB;
    ptB.x = -size;
    ptB.y = size;
    ptB.z = 0;
    geometry_msgs::Point ptC;
    ptC.x = -size;
    ptC.y = -size;
    ptC.z = 0;
    geometry_msgs::Point ptD;
    ptD.x = size;
    ptD.y = -size;
    ptD.z = 0;

    /* borders marker //{ */
    {
      visualization_msgs::Marker borders_marker;
      borders_marker.header = header;

      borders_marker.ns = "borders";
      borders_marker.id = 0;
      borders_marker.type = visualization_msgs::Marker::LINE_LIST;
      borders_marker.action = visualization_msgs::Marker::ADD;

      borders_marker.pose.position.x = pos.x();
      borders_marker.pose.position.y = pos.y();
      borders_marker.pose.position.z = pos.z();

      borders_marker.pose.orientation.x = quat.x();
      borders_marker.pose.orientation.y = quat.y();
      borders_marker.pose.orientation.z = quat.z();
      borders_marker.pose.orientation.w = quat.w();

      borders_marker.scale.x = 0.1;
      borders_marker.scale.y = 0.1;
      borders_marker.scale.z = 0.1;

      borders_marker.color.a = 0.5;  // Don't forget to set the alpha!
      borders_marker.color.r = 0.0;
      borders_marker.color.g = 0.0;
      borders_marker.color.b = 1.0;

      borders_marker.points.push_back(ptA);
      borders_marker.points.push_back(ptB);

      borders_marker.points.push_back(ptB);
      borders_marker.points.push_back(ptC);

      borders_marker.points.push_back(ptC);
      borders_marker.points.push_back(ptD);

      borders_marker.points.push_back(ptD);
      borders_marker.points.push_back(ptA);

      ret.markers.push_back(borders_marker);
    }
    //}

    /* plane marker //{ */
    {
      visualization_msgs::Marker plane_marker;
      plane_marker.header = header;

      plane_marker.ns = "plane";
      plane_marker.id = 1;
      plane_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      plane_marker.action = visualization_msgs::Marker::ADD;

      plane_marker.pose.position.x = pos.x();
      plane_marker.pose.position.y = pos.y();
      plane_marker.pose.position.z = pos.z();

      plane_marker.pose.orientation.x = quat.x();
      plane_marker.pose.orientation.y = quat.y();
      plane_marker.pose.orientation.z = quat.z();
      plane_marker.pose.orientation.w = quat.w();

      plane_marker.scale.x = 1;
      plane_marker.scale.y = 1;
      plane_marker.scale.z = 1;

      plane_marker.color.a = 0.2;  // Don't forget to set the alpha!
      plane_marker.color.r = 0.0;
      plane_marker.color.g = 0.0;
      plane_marker.color.b = 1.0;

      // triangle ABC
      plane_marker.points.push_back(ptA);
      plane_marker.points.push_back(ptB);
      plane_marker.points.push_back(ptC);

      // triangle ACD
      plane_marker.points.push_back(ptA);
      plane_marker.points.push_back(ptC);
      plane_marker.points.push_back(ptD);
      ret.markers.push_back(plane_marker);
    }
    //}

    return ret;
  }
  //}

  /* geometry_msgs::PoseStamped //{ */
  geometry_msgs::PoseStamped BalloonFilter::to_output_message2(const theta_t& plane_theta, const std_msgs::Header& header, const pos_t& origin)
  {
    geometry_msgs::PoseStamped ret;

    const auto pos = plane_origin(plane_theta, origin);
    const auto quat = plane_orientation(plane_theta);

    ret.header = header;
    ret.pose.position.x = pos.x();
    ret.pose.position.y = pos.y();
    ret.pose.position.z = pos.z();
    ret.pose.orientation.w = quat.w();
    ret.pose.orientation.x = quat.x();
    ret.pose.orientation.y = quat.y();
    ret.pose.orientation.z = quat.z();

    return ret;
  }
  //}

  /* nav_msgs::Path //{ */
  nav_msgs::Path BalloonFilter::to_output_message(const std::vector<std::pair<UKF::x_t, ros::Time>>& predictions, const std_msgs::Header& header,
                                                  const theta_t& plane_theta)
  {
    nav_msgs::Path ret;
    ret.header = header;
    ret.poses.reserve(predictions.size());
    const quat_t plane_quat = plane_orientation(plane_theta);

    for (const auto& pred : predictions)
    {
      const UKF::x_t& state = pred.first;
      const ros::Time& timestamp = pred.second;
      const quat_t yaw_quat(Eigen::AngleAxisd(state(ukf::x_yaw), Eigen::Vector3d::UnitZ()));
      const quat_t ori_quat = plane_quat * yaw_quat;
      geometry_msgs::PoseStamped pose;
      pose.header = header;
      pose.header.stamp = timestamp;
      pose.pose.position.x = state.x();
      pose.pose.position.y = state.y();
      pose.pose.position.z = state.z();
      pose.pose.orientation.x = ori_quat.x();
      pose.pose.orientation.y = ori_quat.y();
      pose.pose.orientation.z = ori_quat.z();
      pose.pose.orientation.w = ori_quat.w();
      ret.poses.push_back(pose);
    }

    return ret;
  }
  //}

  /* sensor_msgs::PointCloud2 //{ */
  sensor_msgs::PointCloud2 BalloonFilter::to_output_message(const boost::circular_buffer<pos_t>& points, const std_msgs::Header& header)
  {
    const size_t n_pts = points.size();
    sensor_msgs::PointCloud2 ret;
    ret.header = header;
    ret.height = 1;
    ret.width = n_pts;

    {
      // Prepare the PointCloud2
      sensor_msgs::PointCloud2Modifier modifier(ret);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.reserve(n_pts);
    }

    {
      // Fill the PointCloud2
      sensor_msgs::PointCloud2Iterator<float> iter_x(ret, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(ret, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(ret, "z");
      for (size_t it = 0; it < n_pts; it++, ++iter_x, ++iter_y, ++iter_z)
      {
        *iter_x = points.at(it).x();
        *iter_y = points.at(it).y();
        *iter_z = points.at(it).z();
      }
    }

    return ret;
  }
  //}

  /* balloon_filter::Plane //{ */
  balloon_filter::Plane BalloonFilter::to_output_message(const theta_t& plane_theta)
  {
    balloon_filter::Plane ret;
    const auto norm = plane_theta.block<3, 1>(0, 0).norm();
    const auto normal = plane_theta.block<3, 1>(0, 0) / norm;
    ret.normal.x = normal.x();
    ret.normal.y = normal.y();
    ret.normal.z = normal.z();
    ret.offset = plane_theta(3) / norm;
    return ret;
  }
  //}

  /* balloon_filter::PlaneStamped //{ */
  balloon_filter::PlaneStamped BalloonFilter::to_output_message(const theta_t& plane_theta, const std_msgs::Header& header)
  {
    balloon_filter::PlaneStamped ret;
    ret.header = header;
    ret.plane = to_output_message(plane_theta);
    return ret;
  }
  //}

  /* balloon_filter::UKFState //{ */
  balloon_filter::UKFState BalloonFilter::to_output_message(const UKF::statecov_t& ukf_statecov)
  {
    balloon_filter::UKFState ret;
    ret.position.x = ukf_statecov.x(ukf::x_x);
    ret.position.y = ukf_statecov.x(ukf::x_y);
    ret.position.z = ukf_statecov.x(ukf::x_z);
    ret.yaw = ukf_statecov.x(ukf::x_yaw);
    ret.speed = ukf_statecov.x(ukf::x_s);
    ret.curvature = ukf_statecov.x(ukf::x_c);
    return ret;
  }
  //}

  //}

  /* get_cur_mav_pos() method //{ */
  pos_t BalloonFilter::get_cur_mav_pos()
  {
    Eigen::Affine3d m2w_tf;
    bool tf_ok = get_transform_to_world(m_uav_frame_id, ros::Time::now(), m2w_tf);
    if (!tf_ok)
      return pos_t(0, 0, 0);
    ;
    return m2w_tf.translation();
  }
  //}

  /* find_closest_to() method //{ */
  bool BalloonFilter::find_closest_to(const std::vector<pos_cov_t>& measurements, const pos_t& to_position, pos_cov_t& closest_out, bool use_gating)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    pos_cov_t closest_pt{std::numeric_limits<double>::quiet_NaN() * pos_t::Ones(), std::numeric_limits<double>::quiet_NaN() * cov_t::Ones()};
    for (const auto& pt : measurements)
    {
      const double cur_dist = (to_position - pt.pos).norm();
      if (cur_dist < min_dist)
      {
        min_dist = cur_dist;
        closest_pt = pt;
      }
    }
    if (use_gating)
    {
      if (min_dist < m_gating_distance)
      {
        closest_out = closest_pt;
        return true;
      } else
      {
        return false;
      }
    } else
    {
      closest_out = closest_pt;
      return true;
    }
  }
  //}

  /* find_closest() method //{ */
  bool BalloonFilter::find_closest(const std::vector<pos_cov_t>& measuremets, pos_cov_t& closest_out)
  {
    pos_t cur_pos = get_cur_mav_pos();
    return find_closest_to(measuremets, cur_pos, closest_out, false);
  }
  //}

  /* message_to_positions() method //{ */
  std::vector<pos_cov_t> BalloonFilter::message_to_positions(const detections_t& balloon_msg)
  {
    std::vector<pos_cov_t> ret;

    // Construct a new world to sensor transform
    Eigen::Affine3d s2w_tf;
    bool tf_ok = get_transform_to_world(balloon_msg.header.frame_id, balloon_msg.header.stamp, s2w_tf);
    if (!tf_ok)
      return ret;
    const Eigen::Matrix3d s2w_rot = s2w_tf.rotation();

    ret.reserve(balloon_msg.poses.size());
    for (size_t it = 0; it < balloon_msg.poses.size(); it++)
    {
      const auto msg_pos = balloon_msg.poses[it].pose;
      const auto msg_cov = balloon_msg.poses[it].covariance;
      const pos_t pos = s2w_tf * pos_t(msg_pos.position.x, msg_pos.position.y, msg_pos.position.z);
      if (point_valid(pos))
      {
        const cov_t cov = rotate_covariance(msg2cov(msg_cov), s2w_rot);
        const pos_cov_t pos_cov{pos, cov};
        ret.push_back(pos_cov);
      } else
      {
        ROS_INFO("[%s]: Skipping invalid point [%.2f, %.2f, %.2f] (original: [%.2f %.2f %.2f])", m_node_name.c_str(), pos.x(), pos.y(), pos.z(),
                 msg_pos.position.x, msg_pos.position.y, msg_pos.position.z);
      }
    }

    return ret;
  }
  //}

  /* msg2cov() method //{ */
  cov_t BalloonFilter::msg2cov(const ros_cov_t& msg_cov)
  {
    cov_t cov;
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        cov(r, c) = msg_cov[r * 6 + c];
      }
    }
    return cov;
  }
  //}

  /* rotate_covariance() method //{ */
  cov_t BalloonFilter::rotate_covariance(const cov_t& covariance, const cov_t& rotation)
  {
    return rotation * covariance * rotation.transpose();  // rotate the covariance to point in direction of est. position
  }
  //}

  /* point_valid() method //{ */
  bool BalloonFilter::point_valid(const pos_t& pt)
  {
    const bool height_valid = pt.z() > m_z_bounds_min && pt.z() < m_z_bounds_max;
    const bool sane_values = !pt.array().isNaN().any() && !pt.array().isInf().any();
    return height_valid && sane_values;
  }
  //}

  /* plane_orientation() method //{ */
  quat_t BalloonFilter::plane_orientation(const theta_t& plane_theta)
  {
    const quat_t ret = mrs_lib::quaternion_between({0, 0, 1}, plane_theta.block<3, 1>(0, 0));
    return ret;
  }
  //}

  /* plane_angle() method //{ */
  double BalloonFilter::plane_angle(const theta_t& plane1, const theta_t& plane2)
  {
    const auto normal1 = plane1.block<3, 1>(0, 0).normalized();
    const auto normal2 = plane2.block<3, 1>(0, 0).normalized();
    return mrs_lib::angle_between(normal1, normal2);
  }
  //}

  /* plane_origin() method //{ */
  pos_t BalloonFilter::plane_origin(const theta_t& plane_theta, const pos_t& origin)
  {
    const static double eps = 1e-9;
    const double a = plane_theta(0);
    const double b = plane_theta(1);
    const double c = plane_theta(2);
    const double d = plane_theta(3);
    const double x = origin.x();
    const double y = origin.y();
    const double z = origin.z();
    pos_t ret(x, y, z);
    if (abs(a) > eps)
      ret(0) = -(y * b + z * c + d) / a;
    else if (abs(b) > eps)
      ret(1) = -(x * a + z * c + d) / b;
    else if (abs(c) > eps)
      ret(2) = -(x * a + y * b + d) / c;
    return ret;
  }
  //}

  /* plane_theta_to_ukf_u() method //{ */
  UKF::u_t BalloonFilter::plane_theta_to_ukf_u(const theta_t& plane_theta)
  {
    const quat_t quat = plane_orientation(plane_theta);
    UKF::u_t ret;
    ret(ukf::u_qw) = quat.w();
    ret(ukf::u_qx) = quat.x();
    ret(ukf::u_qy) = quat.y();
    ret(ukf::u_qz) = quat.z();
    return ret;
  }
  //}

  /* load_dynparams() method //{ */
  void BalloonFilter::load_dynparams(drcfg_t cfg)
  {
    m_z_bounds_min = cfg.z_bounds__min;
    m_z_bounds_max = cfg.z_bounds__max;
    m_gating_distance = cfg.gating_distance;
    m_max_time_since_update = cfg.max_time_since_update;
    m_min_updates_to_confirm = cfg.min_updates_to_confirm;

    m_prediction_horizon = cfg.ukf__prediction_horizon;

    m_process_std(ukf::x_x) = m_process_std(ukf::x_y) = m_process_std(ukf::x_z) = cfg.process_std__position;
    m_process_std(ukf::x_yaw) = cfg.process_std__yaw;
    m_process_std(ukf::x_s) = cfg.process_std__speed;
    m_process_std(ukf::x_c) = cfg.process_std__curvature;

    m_init_std(ukf::x_yaw) = cfg.init_std__yaw;
    m_init_std(ukf::x_s) = cfg.init_std__speed;
    m_init_std(ukf::x_c) = cfg.init_std__curvature;
  }
  //}

  /* onInit() //{ */

  void BalloonFilter::onInit()
  {

    ROS_INFO("[%s]: Initializing", m_node_name.c_str());
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    ros::Time::waitForValid();

    /* load parameters //{ */

    // LOAD DYNAMIC PARAMETERS
    ROS_INFO("[%s]: LOADING DYNAMIC PARAMETERS", m_node_name.c_str());
    m_drmgr_ptr = std::make_unique<drmgr_t>(nh, m_node_name);

    ROS_INFO("[%s]: LOADING STATIC PARAMETERS", m_node_name.c_str());
    mrs_lib::ParamLoader pl(nh, m_node_name);

    const double planning_period = pl.load_param2<double>("planning_period");
    pl.load_param("rheiv/fitting_period", m_rheiv_fitting_period);
    pl.load_param("rheiv/min_points", m_rheiv_min_pts);
    pl.load_param("rheiv/max_points", m_rheiv_max_pts);
    pl.load_param("rheiv/visualization_size", m_rheiv_visualization_size);

    pl.load_param("world_frame", m_world_frame);
    pl.load_param("uav_frame_id", m_uav_frame_id);
    pl.load_param("gating_distance", m_gating_distance);
    pl.load_param("max_time_since_update", m_max_time_since_update);
    pl.load_param("min_updates_to_confirm", m_min_updates_to_confirm);
    pl.load_param("ukf/prediction_step", m_prediction_step);
    const double prediction_period = pl.load_param2<double>("ukf/prediction_period");

    if (!pl.loaded_successfully())
    {
      ROS_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
      ros::shutdown();
    }

    if (!m_drmgr_ptr->loaded_successfully())
    {
      ROS_ERROR("Some dynamic parameter default values were not loaded successfully, ending the node");
      ros::shutdown();
    }

    //}

    /* subscribers //{ */

    m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, m_node_name);
    mrs_lib::SubscribeMgr smgr(nh);
    constexpr bool time_consistent = true;
    m_sh_balloons = smgr.create_handler<detections_t, time_consistent>("balloon_detections", ros::Duration(5.0));

    m_reset_chosen_server = nh.advertiseService("reset_chosen", &BalloonFilter::reset_chosen_callback, this);
    //}

    /* publishers //{ */

    m_pub_chosen_balloon = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("filtered_position", 1);
    m_pub_used_meas = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("detection_used", 1);
    m_pub_ball_prediction = nh.advertise<balloon_filter::BallPrediction>("ball_prediction", 1);
    m_pub_pred_path_dbg = nh.advertise<nav_msgs::Path>("predicted_path", 1);
    m_pub_plane_dbg = nh.advertise<visualization_msgs::MarkerArray>("fitted_plane_marker", 1);
    m_pub_plane_dbg2 = nh.advertise<geometry_msgs::PoseStamped>("fitted_plane_pose", 1);
    m_pub_used_pts = nh.advertise<sensor_msgs::PointCloud2>("fit_points", 1);
    m_pub_fitted_plane = nh.advertise<balloon_filter::PlaneStamped>("fitted_plane", 1);

    //}

    /* profiler //{ */

    m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

    //}

    {
      UKF::transition_model_t tra_model(ukf::tra_model_f);
      UKF::observation_model_t obs_model(ukf::obs_model_f);
      m_ukf = UKF(ukf::tra_model_f, ukf::obs_model_f);
    }

    {
      const rheiv::f_z_t f_z(rheiv::f_z_f);
      const rheiv::dzdx_t dzdx = rheiv::dzdx_t::Identity();
      m_rheiv = RHEIV(f_z, dzdx, 1e-15, 1e4);
      m_rheiv_pts.set_capacity(m_rheiv_max_pts);
      m_rheiv_covs.set_capacity(m_rheiv_max_pts);
      m_rheiv_theta_valid = false;
    }

    reset_ukf_estimate();
    m_is_initialized = true;

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonFilter::main_loop, this);
    m_rheiv_loop_timer = nh.createTimer(ros::Duration(m_rheiv_fitting_period), &BalloonFilter::rheiv_loop, this);
    m_prediction_loop_timer = nh.createTimer(ros::Duration(prediction_period), &BalloonFilter::prediction_loop, this);

    //}

    ROS_INFO("[%s]: initialized", m_node_name.c_str());
  }

  //}

  /* BalloonFilter::reset_chosen_callback() method //{ */

  bool BalloonFilter::reset_chosen_callback(balloon_filter::ResetChosen::Request& req, balloon_filter::ResetChosen::Response& resp)
  {
    reset_ukf_estimate();
    resp.message = "Current chosen balloon was reset.";
    resp.success = true;
    return true;
  }

  //}

}  // namespace balloon_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_filter::BalloonFilter, nodelet::Nodelet)
