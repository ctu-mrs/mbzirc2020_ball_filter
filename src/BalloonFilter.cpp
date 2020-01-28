#include <balloon_filter/BalloonFilter.h>

namespace balloon_filter
{

  /* process_detections() method //{ */
  void BalloonFilter::process_detections(const detections_t& detections_msg)
  {
    if (!detections_msg.detections.empty())
    {
      /* ROS_INFO("[%s]: Processing %lu new detections vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv", m_node_name.c_str(), detections_msg.poses.size()); */
  
      // transform the message to usable measurement format
      std::vector<pos_cov_t> measurements = message_to_positions(detections_msg);

      /* choose one or none valid measurement to use //{ */
      
      if (m_prev_measurements.empty())
      {
        m_prev_measurements.push_back({measurements, detections_msg.header.stamp});
        return;
      }
      
      const auto [prev_measurements, prev_measurements_stamp] = find_closest_dt(m_prev_measurements, detections_msg.header.stamp, m_meas_filt_desired_dt);
      const double dt = (detections_msg.header.stamp - prev_measurements_stamp).toSec();
      ROS_INFO_THROTTLE(1.0, "[BalloonFilter]: Using detection with %.2fs dt.", dt);
      const auto chosen_meass_opt = find_speed_compliant_measurement(prev_measurements, measurements, ball_speed_at_time(detections_msg.header.stamp), dt, m_meas_filt_loglikelihood_threshold, m_meas_filt_covariance_inflation);
      m_prev_measurements.push_back({measurements, detections_msg.header.stamp});
      if (!chosen_meass_opt.has_value())
      {
        ROS_WARN_THROTTLE(1.0, "[BalloonFilter]: No detections complied with the expected speed, skipping.");
        return;
      }
      const auto chosen_meas_prev = chosen_meass_opt.value().first;
      const auto chosen_meas = chosen_meass_opt.value().second;
  
      /* publish the results //{ */
      
      {
        std_msgs::Header header;
        header.frame_id = m_world_frame_id;
        header.stamp = detections_msg.header.stamp;

        /* publish the chosen measurement as the dedicated message */
        m_pub_chosen_meas.publish(to_output_message(chosen_meas, header, detections_msg.camera_info));

        /* publish the chosen measurement as PoseWithCovarianceStamped for debugging and visualisation purposes */
        m_pub_chosen_meas_dbg.publish(to_output_message(chosen_meas, header));
      
        /* publish debug output of the measurement choosing filter //{ */
        {
          const size_t n_pts = measurements.size() + prev_measurements.size();
          sensor_msgs::PointCloud2 ret;
          ret.header = header;
          ret.height = 1;
          ret.width = n_pts;
      
          {
            // Prepare the PointCloud2
            sensor_msgs::PointCloud2Modifier modifier(ret);
            modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                             "y", 1, sensor_msgs::PointField::FLOAT32,
                                             "z", 1, sensor_msgs::PointField::FLOAT32,
                                             "type", 1, sensor_msgs::PointField::INT32);
            modifier.resize(n_pts);
          }
      
          {
            // Fill the PointCloud2
            sensor_msgs::PointCloud2Iterator<float> iter_x(ret, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(ret, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(ret, "z");
            sensor_msgs::PointCloud2Iterator<int32_t> iter_type(ret, "type");
            for (size_t it = 0; it < measurements.size(); it++, ++iter_x, ++iter_y, ++iter_z, ++iter_type)
            {
              const auto& pt = measurements.at(it);
              *iter_x = pt.pos.x();
              *iter_y = pt.pos.y();
              *iter_z = pt.pos.z();
              if (pt.pos == chosen_meas.pos && pt.cov == chosen_meas.cov)
                *iter_type = 0;
              else
                *iter_type = 1;
            }
            for (size_t it = 0; it < prev_measurements.size(); it++, ++iter_x, ++iter_y, ++iter_z, ++iter_type)
            {
              const auto& pt = prev_measurements.at(it);
              *iter_x = pt.pos.x();
              *iter_y = pt.pos.y();
              *iter_z = pt.pos.z();
              if (pt.pos == chosen_meas_prev.pos && pt.cov == chosen_meas_prev.cov)
                *iter_type = 2;
              else
                *iter_type = 3;
            }
          }
          m_pub_meas_filt_dbg.publish(ret);
        }
        //}
      }
      
      //}
  
      add_rheiv_data(chosen_meas.pos, chosen_meas.cov, detections_msg.header.stamp);
      
      //}
  
      // copy the latest plane fit
      const auto [plane_theta_valid, plane_theta] = get_mutexed(m_rheiv_theta_mtx, m_rheiv_theta_valid, m_rheiv_theta);
  
      /* try to publish plane debug visualization markers //{ */
      
      if (plane_theta_valid && (m_pub_plane_dbg.getNumSubscribers() > 0 || m_pub_plane_dbg2.getNumSubscribers()))
      {
        std_msgs::Header header;
        header.frame_id = m_world_frame_id;
        header.stamp = ros::Time::now();
        if (m_pub_plane_dbg.getNumSubscribers())
          m_pub_plane_dbg.publish(to_output_message(plane_theta, header, get_pos(m_ukf_estimate.x)));
        if (m_pub_plane_dbg2.getNumSubscribers())
          m_pub_plane_dbg2.publish(to_output_message2(plane_theta, header, get_pos(m_ukf_estimate.x)));
      }
      
      //}
  
      /* update the UKF if possible //{ */
      
      // check if we have all data that is needed
      if (plane_theta_valid)
      {
        /* update the UKF //{ */
      
        if (m_ukf_estimate_exists)
        {
          update_ukf_estimate(chosen_meas, detections_msg.header.stamp, plane_theta);
        }
        else
        {
          init_ukf_estimate(detections_msg.header.stamp, plane_theta);
          lpf_reset(m_ukf_estimate.x, detections_msg.header.stamp);
        }
      
        /* publish the filtered curvature //{ */
      
        {
          const auto filtered = lpf_filter_states(m_ukf_estimate.x, detections_msg.header.stamp);
          mrs_msgs::Float64Stamped msg;
          msg.header = detections_msg.header;
          msg.value = filtered;
          m_pub_lpf.publish(msg);
        }
      
        //}
      
        //}
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
      
      //}
  
      /* update the LKF //{ */
      
      if (m_lkf_estimate_exists)
      {
        update_lkf_estimate(chosen_meas, detections_msg.header.stamp);
      }
      else
      {
        init_lkf_estimate(detections_msg.header.stamp);
      }
      
      //}
  
      ros::Duration del = ros::Time::now() - detections_msg.header.stamp;
      ROS_INFO_STREAM_THROTTLE(MSG_THROTTLE, "[KF]: delay (from image acquisition): " << del.toSec() * 1000.0 << "ms");
      /* ROS_INFO("[%s]: New data processed          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^", m_node_name.c_str()); */
    } else
    {
      ROS_INFO_THROTTLE(MSG_THROTTLE, "[%s]: Empty detections message received", m_node_name.c_str());
    }
  }
  //}

  /* main_loop() method //{ */
  void BalloonFilter::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    static ros::Time prev_stamp = ros::Time::now();
    const ros::Time cur_stamp = ros::Time::now();
    if (cur_stamp < prev_stamp)
    {
      ROS_WARN("[BalloonFilter]: Detected jump back in time, resetting.");
      reset_estimates();
    }
    prev_stamp = cur_stamp;
    load_dynparams(m_drmgr_ptr->config);

    if (m_sh_balloons->new_data())
      process_detections(*m_sh_balloons->get_data());

    if (m_sh_balloons_bfx->new_data())
      process_detections(*m_sh_balloons_bfx->get_data());

    const auto lkf_last_update = get_mutexed(m_lkf_estimate_mtx, m_lkf_last_update);
    if ((ros::Time::now() - lkf_last_update).toSec() >= m_max_time_since_update)
      reset_lkf_estimate();

    const auto ukf_last_update = get_mutexed(m_ukf_estimate_mtx, m_ukf_last_update);
    if ((ros::Time::now() - ukf_last_update).toSec() >= m_max_time_since_update)
      reset_ukf_estimate();

    if (m_ukf_estimate_exists)
    {
      /* check if the current UKF estimate doesn't violate some thresholds, print it //{ */
      
      const auto [plane_theta_valid, plane_theta] = get_mutexed(m_rheiv_theta_mtx, m_rheiv_theta_valid, m_rheiv_theta);
      if (plane_theta_valid)
      {
        const double dt = (ros::Time::now() - m_ukf_last_update).toSec();
        auto cur_estimate = predict_ukf_estimate(m_ukf_estimate, dt, plane_theta, ball_speed_at_time(m_ukf_last_update));
      
        Eigen::IOFormat short_fmt(3);
        cur_estimate.x.format(short_fmt);
        ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[UKF]: Current UKF prediction:" << std::endl
                                                                                << "[\tx\t\ty\t\tz\t\tyaw\t\tspd\t\tcur\t]" << std::endl
                                                                                << "[" << cur_estimate.x.transpose() << "]");
        /* const auto cur_pos_cov = get_pos_cov(cur_estimate); */
        // if the absolute estimated curvature exceeds the user-set threshold, reset it
        const auto curv = std::abs(cur_estimate.x(ukf::x::c));
        if (curv > 0.0)
        {
          const auto radius = 1.0/curv;
          if (radius < m_ukf_min_radius)
          {
            reset_ukf_estimate();
            ROS_WARN("[UKF]: UKF radius (1/%.2f) estimate is under the threshold (%.2f > %.2f), resetting the UKF!", curv, radius, m_ukf_min_radius);
          }
        }
      }
      
      //}
    }
    else if (m_lkf_estimate_exists)
    {
      /* check if the current LKF estimate doesn't violate some thresholds, print it //{ */
      
      ros::Time cur_stamp = ros::Time::now();
      const double dt = (cur_stamp - m_lkf_last_update).toSec();
      auto cur_estimate = predict_lkf_estimate(m_lkf_estimate, dt);
      
      Eigen::IOFormat short_fmt(3);
      cur_estimate.x.format(short_fmt);
      ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[LKF]: Current LKF prediction:" << std::endl
                                                                              << "[\tx\t\ty\t\tz\t\tdx\t\tdy\t\tdz\t]" << std::endl
                                                                              << "[" << cur_estimate.x.transpose() << "]");
      /* const auto cur_pos_cov = get_pos_cov(cur_estimate); */
      // if the absolute estimated curvature exceeds the user-set threshold, reset it
      const auto speed = cur_estimate.x.block<3, 1>(3, 0).norm();
      const auto exp_speed = ball_speed_at_time(cur_stamp);
      const auto speed_err = std::abs(speed - exp_speed);
      ROS_INFO_THROTTLE(1.0, "[LKF]: Current LKF speed estimate is %.2fm/s (expected: %.2f)!", speed, exp_speed);
      if (speed_err > m_lkf_max_speed_err)
      {
        reset_lkf_estimate();
        ROS_WARN("[LKF]: LKF speed estimate error exceeded the threshold (%.2f > %.2f), resetting the LKF!", speed_err, m_lkf_max_speed_err);
      }
      
      //}
    }
  }
  //}

  /* rheiv_loop() method //{ */
  void BalloonFilter::rheiv_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    // threadsafe copy the data to be fitted with the plane
    const auto [rheiv_pts, rheiv_covs, rheiv_new_data, rheiv_last_data_update] = get_set_mutexed(m_rheiv_data_mtx,
        std::forward_as_tuple(m_rheiv_pts, m_rheiv_covs, m_rheiv_new_data, m_rheiv_last_data_update),
        std::make_tuple(false, true),
        std::forward_as_tuple(m_rheiv_new_data, m_rheiv_fitting)
        );

    if ((ros::Time::now() - rheiv_last_data_update).toSec() >= m_max_time_since_update)
    {
      reset_rheiv_estimate();
      return;
    }

    if (!rheiv_new_data)
      return;

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

        {
          std::scoped_lock lck(m_rheiv_data_mtx);
          // check if the fitting was not reset in the meantime
          if (m_rheiv_fitting)
          {
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
          // reset the fitting flag
          m_rheiv_fitting = false;
        }
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
      header.frame_id = m_world_frame_id;
      header.stamp = ros::Time::now();
      m_pub_used_pts.publish(to_output_message(rheiv_pts, header));
    }

    if (success)
    {
      std_msgs::Header header;
      header.frame_id = m_world_frame_id;
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

  /* /1* lpf_loop() method //{ *1/ */
  /* void BalloonFilter::lpf_loop([[maybe_unused]] const ros::TimerEvent& evt) */
  /* { */
  /*   const auto [ukf_estimate_exists, ukf_estimate] = mrs_lib::get_mutexed(m_ukf_estimate_mtx, m_ukf_estimate_exists, m_ukf_estimate); */

  /*   if (!ukf_estimate_exists) */
  /*   const double curvature = m_ukf_estimate.x(ukf::x::c); */
  /*   static double curv_filtered = curvature; */

  /*   mrs_msgs::Float64Stamped msg; */
  /*   msg.header.stamp = ros::Time::now(); */
  /*   msg.value = curv_filtered; */

  /*   m_pub_lpf.publish(msg); */
  /* } */
  /* //} */

  /* prediction_loop() method //{ */
  void BalloonFilter::prediction_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    const auto [lkf_estimate_exists, lkf_estimate, lkf_last_update, lkf_n_updates] = mrs_lib::get_mutexed(m_lkf_estimate_mtx, m_lkf_estimate_exists, m_lkf_estimate, m_lkf_last_update, m_lkf_n_updates);
    const auto [ukf_estimate_exists, ukf_estimate, ukf_last_update, ukf_n_updates] = mrs_lib::get_mutexed(m_ukf_estimate_mtx, m_ukf_estimate_exists, m_ukf_estimate, m_ukf_last_update, m_ukf_n_updates);
    const auto [plane_theta_valid, plane_theta] = mrs_lib::get_mutexed(m_rheiv_theta_mtx, m_rheiv_theta_valid, m_rheiv_theta);

    balloon_filter::BallPrediction message;
    message.header.frame_id = m_world_frame_id;
    message.header.stamp = ros::Time::now();
    
    balloon_filter::Plane fitted_plane;
    balloon_filter::FilterState filter_state;
    const double expected_speed = ball_speed_at_time(message.header.stamp);
    filter_state.expected_speed = expected_speed;
    filter_state.ukf_state.valid = false;
    filter_state.lkf_state.valid = false;
    nav_msgs::Path predicted_path;

    if (lkf_estimate_exists && lkf_n_updates > m_min_updates_to_confirm)
    {
      /* use LKF by default, if available //{ */
      
      const auto predictions = predict_lkf_states(lkf_estimate, lkf_last_update, m_lkf_prediction_horizon, m_lkf_prediction_step, expected_speed);
      message.header.stamp = lkf_last_update;
      filter_state.lkf_state = to_output_message(lkf_estimate);
      filter_state.lkf_state.valid = true;
      predicted_path = to_output_message(predictions, message.header);
      
      //}
    }

    if (ukf_estimate_exists && ukf_n_updates > m_min_updates_to_confirm && plane_theta_valid)
    {
      /* use the UKF prediction instead (more precise), if available //{ */
      
      const auto predictions = predict_ukf_states(ukf_estimate, ukf_last_update, plane_theta, m_ukf_prediction_horizon, m_ukf_prediction_step);
      message.header.stamp = ukf_last_update;
      filter_state.ukf_state = to_output_message(ukf_estimate);
      filter_state.ukf_state.valid = true;
      predicted_path = to_output_message(predictions, message.header, plane_theta);
      
      //}
    }

    if (plane_theta_valid)
    {
      fitted_plane = to_output_message(plane_theta);
      fitted_plane.valid = true;
    }
    
    filter_state.fitted_plane = fitted_plane;
    message.filter_state = filter_state;
    message.predicted_path = predicted_path;
    
    m_pub_pred_path_dbg.publish(predicted_path);
    m_pub_ball_prediction.publish(message);
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

  /* reset_rheiv_estimate() method //{ */
  void BalloonFilter::reset_rheiv_estimate()
  {
    std::scoped_lock lck(m_rheiv_data_mtx);
    m_rheiv_fitting = false;
    m_rheiv_pts.clear();
    m_rheiv_covs.clear();
    m_rheiv_stamps.clear();
    m_rheiv_new_data = false;
    m_rheiv_last_data_update = ros::Time::now();
    m_rheiv_theta_valid = false;

    ROS_WARN("[%s]: RHEIV estimate ==RESET==.", m_node_name.c_str());
  }
  //}

  // --------------------------------------------------------------
  // |                     UKF related methods                    |
  // --------------------------------------------------------------

  /* predict_ukf_estimate() method //{ */
  UKF::statecov_t BalloonFilter::predict_ukf_estimate(const UKF::statecov_t& ukf_estimate, const double dt, const theta_t& plane_theta, const double ball_speed)
  {
    const UKF::Q_t Q = dt * m_ukf_process_std.asDiagonal();
    const UKF::u_t u = construct_u(plane_theta, ball_speed);
    const auto ret = m_ukf.predict(ukf_estimate, u, Q, dt);
    return ret;
  }
  //}

  /* update_ukf_estimate() method //{ */
  void BalloonFilter::update_ukf_estimate(const pos_cov_t& measurement, const ros::Time& stamp, const theta_t& plane_theta)
  {
    auto [ukf_estimate_exists, ukf_estimate, ukf_last_update, ukf_n_updates] = mrs_lib::get_mutexed(m_ukf_estimate_mtx, m_ukf_estimate_exists, m_ukf_estimate, m_ukf_last_update, m_ukf_n_updates);

    ROS_INFO_THROTTLE(MSG_THROTTLE, "[UKF]: Updating current estimate using point [%.2f, %.2f, %.2f]", measurement.pos.x(), measurement.pos.y(),
                      measurement.pos.z());
    const double dt = (stamp - ukf_last_update).toSec();
    if (dt < 0.0)
      return;

    ukf_estimate = predict_ukf_estimate(ukf_estimate, dt, plane_theta, ball_speed_at_time(ukf_last_update));
    ukf_estimate = m_ukf.correct(ukf_estimate, measurement.pos, measurement.cov);
    ukf_last_update = stamp;
    ukf_n_updates++;
    ukf_estimate_exists = true;

    set_mutexed(m_ukf_estimate_mtx,
        std::make_tuple(ukf_estimate_exists, ukf_estimate, ukf_last_update, ukf_n_updates),
        std::forward_as_tuple(m_ukf_estimate_exists, m_ukf_estimate, m_ukf_last_update, m_ukf_n_updates));
  }
  //}

  /* estimate_ukf_initial_state() method //{ */
  std::optional<UKF::statecov_t> BalloonFilter::estimate_ukf_initial_state(const theta_t& plane_theta)
  {
    const auto [rheiv_pts, rheiv_covs, rheiv_stamps] = mrs_lib::get_mutexed(m_rheiv_data_mtx,
        m_rheiv_pts, m_rheiv_covs, m_rheiv_stamps
        );
  
    assert(rheiv_pts.size() == rheiv_covs.size());
    assert(rheiv_pts.size() == rheiv_stamps.size());
    assert(!rheiv_pts.empty());
    const int n_pts = rheiv_pts.size();
  
    ros::Time cur_time = ros::Time::now();
    ros::Time last_stamp;
    std::vector<std::tuple<pos_t, cov_t, ros::Time>> used_meass;
    for (int it = n_pts-1; it >= 0; it--)
    {
      const auto cur_pt = rheiv_pts.at(it);
      const auto cur_cov = rheiv_covs.at(it);
      const auto cur_stamp = rheiv_stamps.at(it);
      if (it == n_pts-1)
        last_stamp = cur_stamp;
      if (cur_time - cur_stamp > m_ukf_init_history_duration)
        break;
      used_meass.push_back({cur_pt, cur_cov, cur_stamp});
    }
  
    // TODO: 
    // * fit plane through points (or use latest fit?)
    // * project points to plane
    // * fit a conic to the points
    // * calculate its curvature and yaw at the last point
    /* const auto theta = m_rheiv_conic.fit(std::begin(used_pts), std::end(used_pts), std::begin(used_covs), std::end(used_covs)); */

    if (used_meass.empty())
    {
      ROS_ERROR("[BalloonFilter]: No recent points available for initial state estimation. Newest point is %.2fs old, need at most %.2fs.", (cur_time - rheiv_stamps.back()).toSec(), m_ukf_init_history_duration.toSec());
      return std::nullopt;
    }
    ROS_INFO("[UKF]: Trying to initialize UKF using %lu/%i points.", used_meass.size(), n_pts);
  
    UKF::statecov_t statecov;
    ros::Time prev_stamp;
    bool statecov_initd = false;

    for (int it = used_meass.size()-1; it >= 0; it--)
    {
      const auto [cur_pt, cur_cov, cur_stamp] = used_meass.at(it);
      if (!statecov_initd)
      {
        statecov.x(ukf::x::x) = cur_pt.x();
        statecov.x(ukf::x::y) = cur_pt.y();
        statecov.x(ukf::x::z) = cur_pt.z();
        statecov.x(ukf::x::yaw) = 0.0;
        statecov.x(ukf::x::c) = 0.0;
        statecov.P = m_ukf_init_std.asDiagonal();
        statecov.P.block<3, 3>(ukf::x::x, ukf::x::x) = cur_cov;
        prev_stamp = cur_stamp;
        statecov_initd = true;
      }
      else
      {
        const double dt = (cur_stamp - prev_stamp).toSec();
        /* assert(dt >= 0.0); */
        if (dt < 0.0)
          continue;

        statecov = predict_ukf_estimate(statecov, dt, plane_theta, ball_speed_at_time(prev_stamp));
        statecov = m_ukf.correct(statecov, cur_pt, cur_cov);
        prev_stamp = cur_stamp;
      }
    }
    return statecov;
  }
  //}

  /* init_ukf_estimate() method //{ */
  void BalloonFilter::init_ukf_estimate(const ros::Time& stamp, const theta_t& plane_theta)
  {
    const auto init_statecov_opt = estimate_ukf_initial_state(plane_theta);
    if (init_statecov_opt.has_value())
    {
      const auto init_statecov = init_statecov_opt.value();
      ROS_INFO("[UKF]: Initializing estimate using point [%.2f, %.2f, %.2f], yaw %.2f and curvature %.2f",
          init_statecov.x(ukf::x::x), init_statecov.x(ukf::x::y), init_statecov.x(ukf::x::z),
          init_statecov.x(ukf::x::yaw), init_statecov.x(ukf::x::c));

      set_mutexed(m_ukf_estimate_mtx,
          std::make_tuple(      init_statecov,  true,                  stamp,             1),
          std::forward_as_tuple(m_ukf_estimate, m_ukf_estimate_exists, m_ukf_last_update, m_ukf_n_updates));
    }
  }
  //}

  /* reset_ukf_estimate() method //{ */
  void BalloonFilter::reset_ukf_estimate()
  {
    set_mutexed(m_ukf_estimate_mtx,
        std::make_tuple(      false,                 ros::Time::now(),  0),
        std::forward_as_tuple(m_ukf_estimate_exists, m_ukf_last_update, m_ukf_n_updates));
    ROS_WARN("[%s]: UKF estimate ==RESET==.", m_node_name.c_str());
  }
  //}

  /* predict_ukf_states() method //{ */
  std::vector<std::pair<UKF::x_t, ros::Time>> BalloonFilter::predict_ukf_states(const UKF::statecov_t initial_statecov, const ros::Time& initial_timestamp,
                                                                            const theta_t& plane_theta, const double prediction_horizon,
                                                                            const double prediction_step)
  {
    assert(prediction_step > 0.0);
    assert(prediction_horizon > 0.0);
    const int n_pts = round(prediction_horizon / prediction_step);

    UKF::statecov_t statecov = initial_statecov;
    ros::Time timestamp = initial_timestamp;
    std::vector<std::pair<UKF::x_t, ros::Time>> ret;
    ret.reserve(n_pts);
    ret.push_back({statecov.x, timestamp});
    for (int it = 0; it < n_pts; it++)
    {
      statecov = predict_ukf_estimate(statecov, prediction_step, plane_theta, ball_speed_at_time(timestamp));
      timestamp += ros::Duration(prediction_step);
      ret.push_back({statecov.x, timestamp});
    }
    return ret;
  }
  //}

  // --------------------------------------------------------------
  // |                     LKF related methods                    |
  // --------------------------------------------------------------

  /* predict_lkf_estimate() method //{ */
  LKF::statecov_t BalloonFilter::predict_lkf_estimate(const LKF::statecov_t& lkf_estimate, const double dt)
  {
    LKF::A_t A(m_lkf_n_states, m_lkf_n_states);
    if (m_lkf_use_acceleration)
      A <<
            1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
            0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
            0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
            0, 0, 0, 1, 0, 0, 1*dt, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 1*dt, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 1*dt,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1
              ;
    else
      A <<
            1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1
              ;
    const LKF::Q_t Q = dt * LKF::Q_t(m_lkf_process_std.asDiagonal()).block(0, 0, m_lkf_n_states, m_lkf_n_states);
    const LKF::u_t u;
    m_lkf.A = A;
    const auto ret = m_lkf.predict(lkf_estimate, u, Q, dt);
    return ret;
  }
  //}

  /* update_lkf_estimate() method //{ */
  void BalloonFilter::update_lkf_estimate(const pos_cov_t& measurement, const ros::Time& stamp)
  {
    auto [lkf_estimate_exists, lkf_estimate, lkf_last_update, lkf_n_updates] = mrs_lib::get_mutexed(m_lkf_estimate_mtx, m_lkf_estimate_exists, m_lkf_estimate, m_lkf_last_update, m_lkf_n_updates);

    ROS_INFO_THROTTLE(MSG_THROTTLE, "[LKF]: Updating current estimate using point [%.2f, %.2f, %.2f]", measurement.pos.x(), measurement.pos.y(),
                      measurement.pos.z());
    const double dt = (stamp - lkf_last_update).toSec();
    if (dt < 0.0)
      return;

    lkf_estimate = predict_lkf_estimate(lkf_estimate, dt);
    lkf_estimate = m_lkf.correct(lkf_estimate, measurement.pos, measurement.cov);
    lkf_last_update = stamp;
    lkf_n_updates++;
    lkf_estimate_exists = true;

    set_mutexed(m_lkf_estimate_mtx,
        std::make_tuple(lkf_estimate_exists, lkf_estimate, lkf_last_update, lkf_n_updates),
        std::forward_as_tuple(m_lkf_estimate_exists, m_lkf_estimate, m_lkf_last_update, m_lkf_n_updates));
  }
  //}

  /* estimate_lkf_initial_state() method //{ */
  std::optional<LKF::statecov_t> BalloonFilter::estimate_lkf_initial_state()
  {
    const auto [rheiv_pts, rheiv_covs, rheiv_stamps] = mrs_lib::get_mutexed(m_rheiv_data_mtx,
        m_rheiv_pts, m_rheiv_covs, m_rheiv_stamps
        );
  
    assert(rheiv_pts.size() == rheiv_covs.size());
    assert(rheiv_pts.size() == rheiv_stamps.size());
    assert(!rheiv_pts.empty());
    const int n_pts = rheiv_pts.size();
    if (n_pts < m_lkf_min_init_points)
    {
      ROS_INFO_THROTTLE(1.0, "[LKF]: Not enough points for LKF initialization: %d/%d", n_pts, m_lkf_min_init_points);
      return std::nullopt;
    }
  
    ros::Time cur_time = ros::Time::now();
    ros::Time last_stamp;
    std::vector<std::tuple<pos_t, cov_t, ros::Time>> used_meass;
    for (int it = n_pts-1; it >= 0; it--)
    {
      const auto cur_pt = rheiv_pts.at(it);
      const auto cur_cov = rheiv_covs.at(it);
      const auto cur_stamp = rheiv_stamps.at(it);
      if (it == n_pts-1)
        last_stamp = cur_stamp;
      if (cur_time - cur_stamp > m_lkf_init_history_duration)
        break;
      used_meass.push_back({cur_pt, cur_cov, cur_stamp});
    }

    if (used_meass.empty())
    {
      ROS_ERROR("[BalloonFilter]: No recent points available for initial state estimation. Newest point is %.2fs old, need at most %.2fs.", (cur_time - rheiv_stamps.back()).toSec(), m_lkf_init_history_duration.toSec());
      return std::nullopt;
    }
    ROS_INFO("[LKF]: Trying to initialize lkf using %lu/%i points.", used_meass.size(), n_pts);
  
    LKF::statecov_t statecov;
    ros::Time prev_stamp;
    bool statecov_initd = false;
    bool speed_initd = false;

    for (int it = used_meass.size()-1; it >= 0; it--)
    {
      const auto [cur_pt, cur_cov, cur_stamp] = used_meass.at(it);
      if (!statecov_initd)
      {
        statecov.x = LKF::x_t::Zero(m_lkf_n_states, 1);
        statecov.x.block<3, 1>(0, 0) = cur_pt;
        statecov.P = LKF::P_t(m_lkf_init_std.asDiagonal()).block(0, 0, m_lkf_n_states, m_lkf_n_states);
        statecov.P.block<3, 3>(0, 0) = cur_cov;
        prev_stamp = cur_stamp;
        statecov_initd = true;
      }
      else
      {
        const double dt = (cur_stamp - prev_stamp).toSec();
        if (dt < 0.0)
          continue;
        if (!speed_initd)
        {
          const pos_t prev_pos = get_pos(statecov.x);
          const pos_t cur_pos = cur_pt;
          const pos_t cur_spd = (cur_pos - prev_pos)/dt;
          statecov.x.block<3, 1>(3, 0) = cur_spd;
          speed_initd = true;
        }
        statecov = predict_lkf_estimate(statecov, dt);
        statecov = m_lkf.correct(statecov, cur_pt, cur_cov);
        prev_stamp = cur_stamp;
      }
    }
    return statecov;
  }
  //}

  /* init_lkf_estimate() method //{ */
  void BalloonFilter::init_lkf_estimate(const ros::Time& stamp)
  {
    const auto init_statecov_opt = estimate_lkf_initial_state();
    if (init_statecov_opt.has_value())
    {
      const auto init_statecov = init_statecov_opt.value();
      if (m_lkf_use_acceleration)
        ROS_INFO("[LKF]: Initializing estimate using point [%.2f, %.2f, %.2f], velocity [%.2f, %.2f, %.2f], acceleration [%.2f, %.2f, %.2f]\n speed: %.2f\n accel: %.2f",
            init_statecov.x(lkf::x::x), init_statecov.x(lkf::x::y), init_statecov.x(lkf::x::z),
            init_statecov.x(lkf::x::dx), init_statecov.x(lkf::x::dy), init_statecov.x(lkf::x::dz),
            init_statecov.x(lkf::x::ddx), init_statecov.x(lkf::x::ddy), init_statecov.x(lkf::x::ddz),
            init_statecov.x.block<3, 1>(3, 0).norm(),
            init_statecov.x.block<3, 1>(6, 0).norm()
            );
      else
        ROS_INFO("[LKF]: Initializing estimate using point [%.2f, %.2f, %.2f], velocity [%.2f, %.2f, %.2f]\n speed: %.2f",
            init_statecov.x(lkf::x::x), init_statecov.x(lkf::x::y), init_statecov.x(lkf::x::z),
            init_statecov.x(lkf::x::dx), init_statecov.x(lkf::x::dy), init_statecov.x(lkf::x::dz),
            init_statecov.x.block<3, 1>(3, 0).norm()
            );

      set_mutexed(m_lkf_estimate_mtx,
          std::make_tuple(      init_statecov,  true,                  stamp,             1),
          std::forward_as_tuple(m_lkf_estimate, m_lkf_estimate_exists, m_lkf_last_update, m_lkf_n_updates));
    }
  }
  //}

  /* reset_lkf_estimate() method //{ */
  void BalloonFilter::reset_lkf_estimate()
  {
    set_mutexed(m_lkf_estimate_mtx,
        std::make_tuple(      false,                 ros::Time::now(),  0),
        std::forward_as_tuple(m_lkf_estimate_exists, m_lkf_last_update, m_lkf_n_updates));
    ROS_WARN("[%s]: LKF estimate ==RESET==.", m_node_name.c_str());
  }
  //}

  /* predict_lkf_states() method //{ */
  std::vector<std::pair<LKF::x_t, ros::Time>> BalloonFilter::predict_lkf_states(const LKF::statecov_t initial_statecov,
                                                                                const ros::Time& initial_timestamp,
                                                                                const double prediction_horizon,
                                                                                const double prediction_step,
                                                                                const double set_speed = std::numeric_limits<double>::quiet_NaN())
  {
    assert(prediction_step > 0.0);
    assert(prediction_horizon > 0.0);
    const int n_pts = round(prediction_horizon / prediction_step);

    LKF::statecov_t statecov = initial_statecov;
    if (!std::isnan(set_speed))
    {
      const double cur_speed = statecov.x.block<3, 1>(3, 0).norm();
      statecov.x.block<3, 1>(3, 0) *= set_speed/cur_speed;
      statecov.P.block<3, 3>(3, 3) *= set_speed/cur_speed;
    }
    ros::Time timestamp = initial_timestamp;
    std::vector<std::pair<LKF::x_t, ros::Time>> ret;
    ret.reserve(n_pts);
    ret.push_back({statecov.x, timestamp});
    for (int it = 0; it < n_pts; it++)
    {
      timestamp += ros::Duration(prediction_step);
      statecov = predict_lkf_estimate(statecov, prediction_step);
      ret.push_back({statecov.x, timestamp});
    }
    return ret;
  }
  //}

  // --------------------------------------------------------------
  // |                     LPF related methods                    |
  // --------------------------------------------------------------

  /* filter_states() method //{ */
  double BalloonFilter::lpf_filter_states(const UKF::x_t& ukf_states, const ros::Time& stamp)
  {
    const double dt = (stamp - m_lpf_last_update).toSec();
    const double omega = M_PI_2*dt*m_lpf_cutoff_freq;
    const double c = std::cos(omega);
    const double alpha = c - 1.0 + std::sqrt(c*c - 4.0*c + 3.0);

    const double curv_curr = ukf_states(ukf::x::c);
    m_curv_filt = (1.0-alpha)*m_curv_filt + alpha*curv_curr;
    ROS_INFO_THROTTLE(1.0, "[LPF]: Using cutoff frequency of %.2fHz for curvature, resulting in %.2f normalized frequency, %.2f its cosine and %.2f alpha.\ncurv_curr: %.2f\ncurv_filt: %.2f", m_lpf_cutoff_freq, omega, c, alpha, curv_curr, m_curv_filt);
    m_lpf_last_update = stamp;

    return m_curv_filt;
  }
  //}

  /* lpf_reset() method //{ */
  void BalloonFilter::lpf_reset(const UKF::x_t& ukf_states, const ros::Time& stamp)
  {
    const double curv_curr = ukf_states(ukf::x::c);
    m_curv_filt = curv_curr;
    m_lpf_last_update = stamp;
  }
  //}

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  /* find_closest_dt() method //{ */
  BalloonFilter::prev_measurement_t BalloonFilter::find_closest_dt(const prev_measurements_t& measurements, const ros::Time& from_time, const ros::Duration& desired_dt)
  {
    assert(!measurements.empty());
    double closest_dt_diff = std::numeric_limits<double>::max();
    prev_measurement_t closest_meas;
    for (const auto& meas : measurements)
    {
      const ros::Time cur_stamp = std::get<1>(meas);
      const ros::Duration cur_dt = from_time - cur_stamp;
      const double cur_dt_diff = (desired_dt - cur_dt).toSec();
      if (cur_dt_diff < closest_dt_diff)
      {
        closest_dt_diff = cur_dt_diff;
        closest_meas = meas;
      }
    }
    return closest_meas;
  }
  //}

  /* reset_estimates() method //{ */
  void BalloonFilter::reset_estimates()
  {
    reset_lkf_estimate();
    reset_ukf_estimate();
    reset_rheiv_estimate();
  }
  //}
  
  /* calc_hyp_meas_loglikelihood() method //{ */
  template <unsigned num_dimensions>
  double BalloonFilter::calc_hyp_meas_loglikelihood(const pos_cov_t& hyp, const pos_cov_t& meas, const double cov_inflation)
  {
    const pos_t inn = meas.pos - hyp.pos;
    const cov_t inn_cov = cov_inflation*(meas.cov + hyp.cov);
    cov_t inverse;
    bool invertible;
    double determinant;
    inn_cov.computeInverseAndDetWithCheck(inverse, determinant, invertible);
    if (!invertible)
      ROS_ERROR("[]: Covariance matrix of a measurement is not invertible!! May produce garbage.");
    constexpr double dylog2pi = num_dimensions*std::log(2*M_PI);
    const double a = inn.transpose() * inverse * inn;
    const double b = std::log(determinant);
    const double res = - (a + b + dylog2pi)/2.0;
    return res;
  }
  //}

  /* find_most_likely_association() method //{ */
  std::tuple<pos_cov_t, double> BalloonFilter::find_most_likely_association(const pos_cov_t& prev_meas, const std::vector<pos_cov_t>& measurements, const double expected_speed, const double dt, const double cov_inflation)
  {
    pos_cov_t most_likely;
    double max_loglikelihood = std::numeric_limits<double>::lowest();
    /* size_t it = 0; */
    for (const auto& meas : measurements)
    {
      const auto diff_vec = meas.pos - prev_meas.pos;
      const auto diff_vec_exp = diff_vec.normalized()*dt*expected_speed;
      const auto err_vec = diff_vec - diff_vec_exp;
      const pos_cov_t err_pos_cov {err_vec, meas.cov};
      const pos_cov_t tmp_pos_cov {{0.0, 0.0, 0.0}, prev_meas.cov};
      const double loglikelihood = calc_hyp_meas_loglikelihood<3>(tmp_pos_cov, err_pos_cov, cov_inflation);
      /* ROS_INFO("[]: loglikelihood %lu: %.2f", it, loglikelihood); it++; */
      if (loglikelihood > max_loglikelihood)
      {
        most_likely = meas;
        max_loglikelihood = loglikelihood;
      }
    }
    return {most_likely, max_loglikelihood};
  }
  //}

  /* find_speed_compliant_measurement() method //{ */
  std::optional<std::pair<pos_cov_t, pos_cov_t>> BalloonFilter::find_speed_compliant_measurement(
      const std::vector<pos_cov_t>& prev_meass,
      const std::vector<pos_cov_t>& measurements,
      const double expected_speed,
      const double dt,
      const double loglikelihood_threshold,
      const double cov_inflation)
  {
    std::optional<pos_cov_t> most_likely = std::nullopt;
    std::optional<pos_cov_t> most_likely_prev = std::nullopt;
    double max_loglikelihood = std::numeric_limits<double>::lowest();
    /* size_t it = 0; */
    for (const auto& prev_meas : prev_meass)
    {
      /* ROS_INFO("[]: Measurement %lu: ------", it); it++; */
      const auto [association, loglikelihood] = find_most_likely_association(prev_meas, measurements, expected_speed, dt, cov_inflation);
      if (loglikelihood > max_loglikelihood)
      {
        most_likely = association;
        most_likely_prev = prev_meas;
        max_loglikelihood = loglikelihood;
      }
    }
    if (most_likely.has_value() && most_likely_prev.has_value())
    {
      if (max_loglikelihood > loglikelihood_threshold)
      {
        ROS_INFO_THROTTLE(1.0, "[BalloonFilter]: Picking measurement with likelihood %.2f", max_loglikelihood);
        ROS_DEBUG("[BalloonFilter]: Picking measurement with likelihood %.2f", max_loglikelihood);
      }
      else
      {
        ROS_INFO_THROTTLE(1.0, "[BalloonFilter]: No measurement sufficiently likely (most likely is %.2f)", max_loglikelihood);
        ROS_DEBUG("[BalloonFilter]: No measurement sufficiently likely (most likely is %.2f)", max_loglikelihood);
        return std::nullopt;
      }
    } else
    {
      ROS_INFO("[BalloonFilter]: No measurement available!");
      return std::nullopt;
    }
    return std::make_pair(most_likely_prev.value(), most_likely.value());
  }
  //}

  /* get_pos() method //{ */
  template <class T>
  pos_t BalloonFilter::get_pos(const T& x)
  {
    return x.template block<3, 1>(0, 0);
  }
  //}

  /* get_pos_cov() method //{ */
  template <class T>
  pos_cov_t BalloonFilter::get_pos_cov(const T& statecov)
  {
    pos_cov_t ret;
    ret.pos = get_pos(statecov.x);
    ret.cov = statecov.P.template block<3, 3>(0, 0);
    return ret;
  }
  //}

  /* to_output_message() method overloads //{ */

  /* balloon_filter::BallLocation //{ */
  balloon_filter::BallLocation BalloonFilter::to_output_message(const pos_cov_t& estimate, const std_msgs::Header& header, const sensor_msgs::CameraInfo& cinfo)
  {
    balloon_filter::BallLocation ret;

    ret.header = header;
    ret.camera_info = cinfo;
    ret.detection.pose.position.x = estimate.pos.x();
    ret.detection.pose.position.y = estimate.pos.y();
    ret.detection.pose.position.z = estimate.pos.z();
    ret.detection.pose.orientation.x = 0.0;
    ret.detection.pose.orientation.y = 0.0;
    ret.detection.pose.orientation.z = 0.0;
    ret.detection.pose.orientation.w = 1.0;

    for (int r = 0; r < 6; r++)
    {
      for (int c = 0; c < 6; c++)
      {
        if (r < 3 && c < 3)
          ret.detection.covariance[r * 6 + c] = estimate.cov(r, c);
        else if (r == c)
          ret.detection.covariance[r * 6 + c] = 666;
        else
          ret.detection.covariance[r * 6 + c] = 0.0;
      }
    }

    return ret;
  }
  //}

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
      const quat_t yaw_quat(Eigen::AngleAxisd(state(ukf::x::yaw), Eigen::Vector3d::UnitZ()));
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

  /* nav_msgs::Path //{ */
  nav_msgs::Path BalloonFilter::to_output_message(const std::vector<std::pair<LKF::x_t, ros::Time>>& predictions, const std_msgs::Header& header)
  {
    nav_msgs::Path ret;
    ret.header = header;
    ret.poses.reserve(predictions.size());

    for (const auto& pred : predictions)
    {
      const LKF::x_t& state = pred.first;
      const ros::Time& timestamp = pred.second;
      const mrs_lib::vec3_t velocity = state.block<3, 1>(3, 0);
      const quat_t quat = mrs_lib::quaternion_between(mrs_lib::vec3_t(1.0, 0.0, 0.0), velocity);
      geometry_msgs::PoseStamped pose;
      pose.header = header;
      pose.header.stamp = timestamp;
      pose.pose.position.x = state.x();
      pose.pose.position.y = state.y();
      pose.pose.position.z = state.z();
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();
      pose.pose.orientation.w = quat.w();
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
      modifier.resize(n_pts);
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

  /* balloon_filter::LKFState //{ */
  balloon_filter::LKFState BalloonFilter::to_output_message(const LKF::statecov_t& lkf_statecov)
  {
    balloon_filter::LKFState ret;
    ret.position.x = lkf_statecov.x(lkf::x::x);
    ret.position.y = lkf_statecov.x(lkf::x::y);
    ret.position.z = lkf_statecov.x(lkf::x::z);
    ret.velocity.x = lkf_statecov.x(lkf::x::dx);
    ret.velocity.y = lkf_statecov.x(lkf::x::dy);
    ret.velocity.z = lkf_statecov.x(lkf::x::dz);
    return ret;
  }
  //}

  /* balloon_filter::UKFState //{ */
  balloon_filter::UKFState BalloonFilter::to_output_message(const UKF::statecov_t& ukf_statecov)
  {
    balloon_filter::UKFState ret;
    ret.position.x = ukf_statecov.x(ukf::x::x);
    ret.position.y = ukf_statecov.x(ukf::x::y);
    ret.position.z = ukf_statecov.x(ukf::x::z);
    ret.yaw = ukf_statecov.x(ukf::x::yaw);
    /* ret.speed = ukf_statecov.x(ukf::x::s); */
    ret.curvature = ukf_statecov.x(ukf::x::c);
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

  /* /1* find_closest_to() method //{ *1/ */
  /* bool BalloonFilter::find_closest_to(const std::vector<pos_cov_t>& measurements, const pos_t& to_position, pos_cov_t& closest_out, bool use_gating) */
  /* { */
  /*   double min_dist = std::numeric_limits<double>::infinity(); */
  /*   pos_cov_t closest_pt{std::numeric_limits<double>::quiet_NaN() * pos_t::Ones(), std::numeric_limits<double>::quiet_NaN() * cov_t::Ones()}; */
  /*   for (const auto& pt : measurements) */
  /*   { */
  /*     const double cur_dist = (to_position - pt.pos).norm(); */
  /*     if (cur_dist < min_dist) */
  /*     { */
  /*       min_dist = cur_dist; */
  /*       closest_pt = pt; */
  /*     } */
  /*   } */
  /*   if (use_gating) */
  /*   { */
  /*     if (min_dist < m_gating_distance) */
  /*     { */
  /*       closest_out = closest_pt; */
  /*       return true; */
  /*     } else */
  /*     { */
  /*       return false; */
  /*     } */
  /*   } else */
  /*   { */
  /*     closest_out = closest_pt; */
  /*     return true; */
  /*   } */
  /* } */
  /* //} */

  /* /1* find_closest() method //{ *1/ */
  /* bool BalloonFilter::find_closest(const std::vector<pos_cov_t>& measuremets, pos_cov_t& closest_out) */
  /* { */
  /*   pos_t cur_pos = get_cur_mav_pos(); */
  /*   return find_closest_to(measuremets, cur_pos, closest_out, false); */
  /* } */
  /* //} */

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

    ret.reserve(balloon_msg.detections.size());
    for (const auto& det : balloon_msg.detections)
    {
      const auto msg_pos = det.pose.pose;
      const auto msg_cov = det.pose.covariance;
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
    const bool in_safety_zone = !m_safety_zone || m_safety_zone->isPointValid(pt.x(), pt.y());
    return height_valid && sane_values && in_safety_zone;
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

  /* construct_u() method //{ */
  UKF::u_t BalloonFilter::construct_u(const theta_t& plane_theta, const double speed)
  {
    const quat_t quat = plane_orientation(plane_theta);
    UKF::u_t ret;
    ret(ukf::u::s) = speed;
    ret(ukf::u::qw) = quat.w();
    ret(ukf::u::qx) = quat.x();
    ret(ukf::u::qy) = quat.y();
    ret(ukf::u::qz) = quat.z();
    return ret;
  }
  //}

  /* ball_speed_at_time() method //{ */
  double BalloonFilter::ball_speed_at_time(const ros::Time& timestamp)
  {
    if (timestamp < m_ball_speed_change)
      return m_ball_speed1;
    else
      return m_ball_speed2;
  }
  //}

  /* load_dynparams() method //{ */
  void BalloonFilter::load_dynparams(drcfg_t cfg)
  {
    m_z_bounds_min = cfg.z_bounds__min;
    m_z_bounds_max = cfg.z_bounds__max;
    m_meas_filt_desired_dt = ros::Duration(cfg.meas_filt__desired_dt);
    m_meas_filt_loglikelihood_threshold = cfg.meas_filt__loglikelihood_threshold;
    m_meas_filt_covariance_inflation = cfg.meas_filt__covariance_inflation;
    m_max_time_since_update = cfg.max_time_since_update;
    m_min_updates_to_confirm = cfg.min_updates_to_confirm;

    /* UKF-related //{ */
    
    m_ukf_prediction_horizon = cfg.ukf__prediction_horizon;
    m_ukf_min_radius = cfg.ukf__min_radius;
    
    m_ukf_process_std(ukf::x::x) = m_ukf_process_std(ukf::x::y) = m_ukf_process_std(ukf::x::z) = cfg.ukf__process_std__position;
    m_ukf_process_std(ukf::x::yaw) = cfg.ukf__process_std__yaw;
    /* m_ukf_process_std(ukf::x_s) = cfg.process_std__speed; */
    m_ukf_process_std(ukf::x::c) = cfg.ukf__process_std__curvature;
    
    m_ukf_init_std(ukf::x::yaw) = cfg.ukf__init_std__yaw;
    /* m_init_std(ukf::x_s) = cfg.init_std__speed; */
    m_ukf_init_std(ukf::x::c) = cfg.ukf__init_std__curvature;
    
    //}

    /* LKF-related //{ */
    
    m_lkf_prediction_horizon = cfg.lkf__prediction_horizon;
    m_lkf_max_speed_err = cfg.lkf__max_speed_err;
    
    m_lkf_process_std(lkf::x::x) = m_lkf_process_std(lkf::x::y) = m_lkf_process_std(lkf::x::z) = cfg.lkf__process_std__position;
    m_lkf_process_std(lkf::x::dx) = m_lkf_process_std(lkf::x::dy) = m_lkf_process_std(lkf::x::dz) = cfg.lkf__process_std__velocity;
    if (m_lkf_use_acceleration)
      m_lkf_process_std(lkf::x::ddx) = m_lkf_process_std(lkf::x::ddy) = m_lkf_process_std(lkf::x::ddz) = cfg.lkf__process_std__acceleration;
    
    m_lkf_init_std(lkf::x::dx) = m_lkf_init_std(lkf::x::dy) = m_lkf_init_std(lkf::x::dz) = cfg.lkf__init_std__velocity;
    if (m_lkf_use_acceleration)
      m_lkf_init_std(lkf::x::ddx) = m_lkf_init_std(lkf::x::ddy) = m_lkf_init_std(lkf::x::ddz) = cfg.lkf__init_std__acceleration;
    
    //}

    m_lpf_cutoff_freq = cfg.lpf__cutoff_freq__curvature;
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

    const int measurements_buffer_length = pl.load_param2<int>("meas_filt/buffer_length");

    const double planning_period = pl.load_param2<double>("planning_period");
    const double prediction_period = pl.load_param2<double>("prediction_period");
    pl.load_param("world_frame_id", m_world_frame_id);
    pl.load_param("uav_frame_id", m_uav_frame_id);
    pl.load_param("ball_speed1", m_ball_speed1);
    pl.load_param("ball_speed2", m_ball_speed2);

    const ros::Duration ball_speed_change_after = pl.load_param2("ball_speed_change_after");
    // TODO: set the time in some smarter manner
    m_ball_speed_change = ros::Time::now() + ball_speed_change_after;
    pl.load_param("rheiv/fitting_period", m_rheiv_fitting_period);
    pl.load_param("rheiv/min_points", m_rheiv_min_pts);
    pl.load_param("rheiv/max_points", m_rheiv_max_pts);
    pl.load_param("rheiv/visualization_size", m_rheiv_visualization_size);
    const double rheiv_timeout_s = pl.load_param2<double>("rheiv/timeout");

    pl.load_param("ukf/init_history_duration", m_ukf_init_history_duration);
    pl.load_param("ukf/prediction_step", m_ukf_prediction_step);

    pl.load_param("lkf/use_acceleration", m_lkf_use_acceleration);
    pl.load_param("lkf/min_init_points", m_lkf_min_init_points);
    pl.load_param("lkf/init_history_duration", m_lkf_init_history_duration);
    pl.load_param("lkf/prediction_step", m_lkf_prediction_step);

    // | ----------------------- safety area ---------------------- |
    /*  //{ */
    
    const auto use_safety_area = pl.load_param2<bool>("safety_area/use_safety_area");
    const auto safety_area_frame = pl.load_param2<std::string>("safety_area/frame_name");
    
    if (use_safety_area)
    {
      const Eigen::MatrixXd border_points = pl.load_matrix_dynamic2("safety_area/safety_area", -1, 2);
    
      const auto obstacle_polygons_enabled = pl.load_param2<bool>("safety_area/polygon_obstacles/enabled");
      std::vector<Eigen::MatrixXd> polygon_obstacle_points;
      if (obstacle_polygons_enabled)
       polygon_obstacle_points = pl.load_matrix_array2("safety_area/polygon_obstacles", std::vector<Eigen::MatrixXd>{});
      else
        polygon_obstacle_points = std::vector<Eigen::MatrixXd>{};
    
      const auto obstacle_points_enabled = pl.load_param2<bool>("safety_area/point_obstacles/enabled");
      std::vector<Eigen::MatrixXd> point_obstacle_points;
      if (obstacle_points_enabled)
        point_obstacle_points = pl.load_matrix_array2("safety_area/point_obstacles", std::vector<Eigen::MatrixXd>{});
      else
        point_obstacle_points = std::vector<Eigen::MatrixXd>{};
    
      // TODO: remove this when param loader supports proper loading
      for (auto& matrix : polygon_obstacle_points)
        matrix.transposeInPlace();

      try
      {
        m_safety_zone = std::make_unique<mrs_lib::SafetyZone>(border_points, polygon_obstacle_points, point_obstacle_points);
      }
      catch (mrs_lib::SafetyZone::BorderError)
      {
        ROS_ERROR("[ControlManager]: Exception caught. Wrong configruation for the safety zone border polygon.");
        ros::shutdown();
      }
      catch (mrs_lib::SafetyZone::PolygonObstacleError)
      {
        ROS_ERROR("[ControlManager]: Exception caught. Wrong configuration for one of the safety zone polygon obstacles.");
        ros::shutdown();
      }
      catch (mrs_lib::SafetyZone::PointObstacleError)
      {
        ROS_ERROR("[ControlManager]: Exception caught. Wrong configuration for one of the safety zone point obstacles.");
        ros::shutdown();
      }
    }
    
    //}

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
    constexpr bool time_consistent = false;
    m_sh_balloons = smgr.create_handler<detections_t, time_consistent>("detections", ros::Duration(5.0));
    m_sh_balloons_bfx = smgr.create_handler<detections_t, time_consistent>("detections_bfx", ros::Duration(5.0));

    m_reset_estimates_server = nh.advertiseService("reset_estimates", &BalloonFilter::reset_estimates_callback, this);
    //}

    /* publishers //{ */

    m_pub_meas_filt_dbg = nh.advertise<sensor_msgs::PointCloud2>("measurement_filter", 1);
    m_pub_chosen_meas = nh.advertise<balloon_filter::BallLocation>("chosen_measurement", 1);
    m_pub_chosen_meas_dbg = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("chosen_measurement_dbg", 1);

    m_pub_plane_dbg = nh.advertise<visualization_msgs::MarkerArray>("fitted_plane_marker", 1);
    m_pub_plane_dbg2 = nh.advertise<geometry_msgs::PoseStamped>("fitted_plane_pose", 1);
    m_pub_used_pts = nh.advertise<sensor_msgs::PointCloud2>("fit_points", 1);
    m_pub_fitted_plane = nh.advertise<balloon_filter::PlaneStamped>("fitted_plane", 1);

    m_pub_ball_prediction = nh.advertise<balloon_filter::BallPrediction>("ball_prediction", 1);
    m_pub_pred_path_dbg = nh.advertise<nav_msgs::Path>("predicted_path", 1);

    m_pub_lpf = nh.advertise<mrs_msgs::Float64Stamped>("low_pass_filtered", 1);

    //}

    /* profiler //{ */

    m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

    //}

    /* initialize UKF //{ */
    {
      
      UKF::transition_model_t tra_model(ukf::tra_model_f);
      UKF::observation_model_t obs_model(ukf::obs_model_f);
      m_ukf = UKF(ukf::tra_model_f, ukf::obs_model_f);
      
    }
    //}

    /* initialize LKF //{ */
    {
      
      if (m_lkf_use_acceleration)
        m_lkf_n_states = 9;
      else
        m_lkf_n_states = 6;
      const LKF::A_t A;
      const LKF::B_t B;
      const LKF::H_t H = LKF::H_t::Identity(lkf::n_measurements, m_lkf_n_states);
      m_lkf = LKF(A, B, H);
      m_lkf_process_std = Eigen::VectorXd(m_lkf_n_states);
      m_lkf_init_std = Eigen::VectorXd(m_lkf_n_states);
      m_lkf_estimate.x = Eigen::VectorXd(m_lkf_n_states);
      m_lkf_estimate.P = Eigen::MatrixXd(m_lkf_n_states, m_lkf_n_states);
      
    }
    //}

    /* initialize RHEIV (the plane-fitting algorithm) //{ */
    {
      
      const rheiv::f_z_t f_z(rheiv::f_z_f);
      const rheiv::dzdx_t dzdx = rheiv::dzdx_t::Identity();
      const std::chrono::duration<double> rheiv_timeout(rheiv_timeout_s*1000);
      m_rheiv = RHEIV(f_z, dzdx, 1e-15, 1e4, rheiv_timeout);
      
      /* const rheiv_conic::f_z_t f_z_conic(rheiv_conic::f_z_f); */
      /* const rheiv_conic::f_dzdx_t f_dzdx_conic (rheiv_conic::f_dzdx_f); */
      /* m_rheiv_conic = RHEIV_conic(f_z_conic, f_dzdx_conic, 1e-9, 1e3); */
      
      m_rheiv_pts.set_capacity(m_rheiv_max_pts);
      m_rheiv_covs.set_capacity(m_rheiv_max_pts);
      m_rheiv_stamps.set_capacity(m_rheiv_max_pts);
      m_rheiv_theta_valid = false;
      
    }
    //}

    m_prev_measurements.set_capacity(measurements_buffer_length);
    reset_estimates();
    m_is_initialized = true;

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonFilter::main_loop, this);
    m_rheiv_loop_timer = nh.createTimer(ros::Duration(m_rheiv_fitting_period), &BalloonFilter::rheiv_loop, this);
    m_prediction_loop_timer = nh.createTimer(ros::Duration(prediction_period), &BalloonFilter::prediction_loop, this);

    //}

    ROS_INFO("[%s]: initialized", m_node_name.c_str());
  }

  //}

  /* BalloonFilter::reset_estimates_callback() method //{ */

  bool BalloonFilter::reset_estimates_callback([[maybe_unused]] balloon_filter::ResetEstimates::Request& req, balloon_filter::ResetEstimates::Response& resp)
  {
    reset_estimates();
    resp.message = "Current estimates were reset.";
    resp.success = true;
    return true;
  }

  //}

}  // namespace balloon_filter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_filter::BalloonFilter, nodelet::Nodelet)
