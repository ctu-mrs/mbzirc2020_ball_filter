#include <balloon_filter/BalloonFilter.h>

namespace balloon_filter
{

  /* main_loop() method //{ */
  void BalloonFilter::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    if (!m_is_initialized)
      return;
    static ros::Time prev_stamp = ros::Time::now();
    const ros::Time cur_stamp = ros::Time::now();
    if (cur_stamp < prev_stamp)
    {
      ROS_WARN("[BalloonFilter]: Detected jump back in time, resetting.");
      reset_estimates();
    }
    prev_stamp = cur_stamp;
    load_dynparams(m_drmgr_ptr->config);

    if (m_sh_localized->new_data())
      process_measurement(*(m_sh_localized->get_data()));

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
          const auto radius = 1.0 / curv;
          if (radius < m_circle_min_radius)
          {
            reset_ukf_estimate();
            ROS_WARN("[UKF]: UKF radius (1/%.2f) estimate is under the threshold (%.2f > %.2f), resetting the UKF!", curv, radius, m_circle_min_radius);
          }
        }
      }

      //}
    } else if (m_lkf_estimate_exists)
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
    if (!m_is_initialized)
      return;
    // threadsafe copy the data to be fitted with the plane
    const auto [rheiv_pts, rheiv_covs, rheiv_new_data, rheiv_last_data_update] =
        get_set_mutexed(m_rheiv_data_mtx, std::forward_as_tuple(m_rheiv_pts, m_rheiv_covs, m_rheiv_new_data, m_rheiv_last_data_update),
                        std::make_tuple(false, true), std::forward_as_tuple(m_rheiv_new_data, m_rheiv_fitting));

    if ((ros::Time::now() - rheiv_last_data_update).toSec() >= m_max_time_since_update)
    {
      reset_rheiv_estimate();
      reset_circle_estimate();
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

      pc_XYZ_t::Ptr pointcloud = boost::make_shared<pc_XYZ_t>();
      pointcloud->reserve(rheiv_pts.size());
      for (const auto& pt : rheiv_pts)
      {
        pt_XYZ_t pclpt(pt.x(), pt.y(), pt.z());
        pointcloud->push_back(pclpt);
      }

      bool well_conditioned = false;
      // First try to fit a line and check how many points are left.
      // This is to make sure that the point set is well conditioned for plane fitting.
      /*  //{ */

      {
        auto model_l = boost::make_shared<pcl::SampleConsensusModelLine<pt_XYZ_t>>(pointcloud);
        pcl::RandomSampleConsensus<pt_XYZ_t> fitter(model_l);
        fitter.setDistanceThreshold(2.0);
        fitter.computeModel();
        Eigen::VectorXf params;
        fitter.getModelCoefficients(params);
        std::vector<int> inliers;
        fitter.getInliers(inliers);

        const double line_pts_ratio = inliers.size() / double(pointcloud->size());
        well_conditioned = line_pts_ratio < m_rheiv_max_line_pts_ratio;
        if (well_conditioned)
          ROS_INFO_THROTTLE(MSG_THROTTLE, "[RHEIV]: Point set is well conditioned (ratio of line points is %.2f < %.2f), fitting plane.", line_pts_ratio,
                            m_rheiv_max_line_pts_ratio);
        else
          ROS_WARN_THROTTLE(MSG_THROTTLE, "[RHEIV]: Point set is NOT well conditioned (ratio of line points is %.2f >= %.2f), NOT fitting plane.",
                            line_pts_ratio, m_rheiv_max_line_pts_ratio);
      }

      //}

      // only continue if the point set is well conditioned for plane fitting
      if (well_conditioned)
      {
        // actually try to fit the plane
        /*  //{ */

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
              ROS_INFO_STREAM_THROTTLE(MSG_THROTTLE, "[RHEIV]: Fitted new plane estimate to " << rheiv_pts.size() << " points in "
                                                                                              << (stamp - fit_time_start).toSec()
                                                                                              << "s (angle diff: " << angle_diff << "):" << std::endl
                                                                                              << "[" << theta.transpose() << "]");
            }
            // reset the fitting flag
            m_rheiv_fitting = false;
          }
        }  // try
        catch (const mrs_lib::eigenvector_exception& ex)
        {
          // Fitting threw exception, notify the user.
          stamp = ros::Time::now();
          ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[RHEIV]: Could not fit plane: '" << ex.what() << "' (took " << (stamp - fit_time_start).toSec() << "s).");
        }

        //}
      }  // if (well_conditioned)

      std_msgs::Header header;
      header.frame_id = m_world_frame_id;
      header.stamp = stamp;

      if (success)
      {
        // plane fit is now available - try to fit a circle to the points to find the curvature
        /*  //{ */

        {
          // align plane points to the XY plane
          const Eigen::Vector3f normal = theta.block<3, 1>(0, 0).cast<float>();
          const float d = -theta(3);
          const Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(normal, Eigen::Vector3f::UnitZ());
          const Eigen::Vector3f offset_pt = normal*d/(normal.dot(normal));

          for (auto& pt : pointcloud->points)
            pt.getVector3fMap() = quat * (pt.getVector3fMap() - offset_pt);
          pointcloud->header.frame_id = m_world_frame_id;
          m_pub_pcl_dbg.publish(pointcloud);

          // fit a 2D circle to the points
          auto model_c2d = boost::make_shared<pcl::SampleConsensusModelCircle2D<pt_XYZ_t>>(pointcloud);
          pcl::RandomSampleConsensus<pt_XYZ_t> fitter(model_c2d);
          fitter.setDistanceThreshold(m_circle_fit_threshold_distance);
          fitter.computeModel();
          Eigen::VectorXf params;
          fitter.getModelCoefficients(params);
          std::vector<int> inliers;
          fitter.getInliers(inliers);

          if (params.size() == 3)
          {
            const float radius = std::abs(params(2));
            if (radius < m_circle_min_radius || radius > m_circle_max_radius)
            {
              ROS_WARN_THROTTLE(
                  MSG_THROTTLE,
                  "[RHEIV]: Fitted INVALID circle with radius %.2fm (not between %.2fm and %.2fm) to points on the plane (%lus/%lus points are inliers).",
                  m_circle_min_radius, m_circle_max_radius, radius, inliers.size(), pointcloud->size());
            }
            // otherwise the circle is valid - hooray! inform the user and save it's state
            else
            {
              ROS_INFO_THROTTLE(MSG_THROTTLE, "[RHEIV]: Fitted circle with radius %.2fm to points on the plane (%lus/%lus points are inliers).", radius,
                                inliers.size(), pointcloud->size());

              {
                std::scoped_lock lck(m_circle_mtx);
                m_circle_valid = true;
                m_circle.center = (quat.inverse()*Eigen::Vector3f{params(0), params(1), 0} + offset_pt).cast<double>();
                m_circle.normal = normal.normalized().cast<double>();
                m_circle.radius = params(2);
              }

              const auto msg = circle_visualization(m_circle, header);
              m_pub_circle_dbg.publish(msg);

            }
          } else
          {
            ROS_WARN_THROTTLE(MSG_THROTTLE, "[RHEIV]: Failed to fit a circle to points on the plane (using %lu points).", pointcloud->size());
          }
        }

        //}

        // publish the results
        m_pub_fitted_plane.publish(to_output_message(theta, header));
      }
    } else  // if (rheiv_pts.size() > (size_t)m_rheiv_min_pts)
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
    if (!m_is_initialized)
      return;
    const auto [lkf_estimate_exists, lkf_estimate, lkf_last_update, lkf_n_updates] =
        mrs_lib::get_mutexed(m_lkf_estimate_mtx, m_lkf_estimate_exists, m_lkf_estimate, m_lkf_last_update, m_lkf_n_updates);
    const auto [ukf_estimate_exists, ukf_estimate, ukf_last_update, ukf_n_updates] =
        mrs_lib::get_mutexed(m_ukf_estimate_mtx, m_ukf_estimate_exists, m_ukf_estimate, m_ukf_last_update, m_ukf_n_updates);
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
    predicted_path.header = message.header;

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

  /* process_measurement() method //{ */
  void BalloonFilter::process_measurement(const geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    if (msg.header.frame_id != m_world_frame_id)
    {
      ROS_ERROR("[BalloonFilter]: Message is in wrong frame: '%s' (expected '%s'). Skipping!", msg.header.frame_id.c_str(), m_world_frame_id.c_str());
      return;
    }

    pose_cov_t det_pose;
    det_pose.pos_cov.pos.x() = msg.pose.pose.position.x;
    det_pose.pos_cov.pos.y() = msg.pose.pose.position.y;
    det_pose.pos_cov.pos.z() = msg.pose.pose.position.z;
    det_pose.pos_cov.cov = msg2cov(msg.pose.covariance);

    // check if the direction covariance is sufficiently low to use it
    if (msg.pose.covariance.at(6*6-1) < M_PI_2)
    {
      quat_t quat;
      quat.w() = msg.pose.pose.orientation.w;
      quat.x() = msg.pose.pose.orientation.x;
      quat.y() = msg.pose.pose.orientation.y;
      quat.z() = msg.pose.pose.orientation.z;
      const cov_t ypr_cov = msg2cov(msg.pose.covariance, 3);
      det_pose.ori_cov = {quat, ypr_cov};
    }

    add_rheiv_data(det_pose.pos_cov.pos, det_pose.pos_cov.cov, msg.header.stamp);

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
        update_ukf_estimate(det_pose, msg.header.stamp, plane_theta);
      } else
      {
        init_ukf_estimate(msg.header.stamp, plane_theta);
      }

      //}

      const auto [circle_valid, circle3d] = get_mutexed(m_circle_mtx, m_circle_valid, m_circle);
      // if we have a fitted circle, use it to correct the UKF estimated curvature
      if (circle_valid)
        update_ukf_estimate(circle3d, plane_theta);

    } else
    {
      if (!plane_theta_valid)
      {
        ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[UKF] RHEIV plane theta estimate unavailable, cannot update UKF!");
      }
    }  // if (!measurements.empty() && plane_theta_valid)

    //}

    /* update the LKF //{ */

    if (m_lkf_estimate_exists)
    {
      update_lkf_estimate(det_pose, msg.header.stamp);
    } else
    {
      init_lkf_estimate(msg.header.stamp);
    }

    //}

    ros::Duration del = ros::Time::now() - msg.header.stamp;
    ROS_INFO_STREAM_THROTTLE(MSG_THROTTLE, "[KF]: delay (from image acquisition): " << del.toSec() * 1000.0 << "ms");
    /* ROS_INFO("[%s]: New data processed          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^", m_node_name.c_str()); */
  }
  //}

  /* init_safety_area() method //{ */
  void BalloonFilter::init_safety_area([[maybe_unused]] const ros::TimerEvent& evt)
  {
    assert(m_safety_area_border_points.cols() == 2);
    const auto tf_opt = m_transformer.getTransform(m_safety_area_frame, m_world_frame_id);
    if (!tf_opt.has_value())
    {
      ROS_ERROR("Safety area could not be transformed!");
      return;
    }

    const auto tf = tf_opt.value();
    /* transform border_points //{ */

    {
      for (int it = 0; it < m_safety_area_border_points.rows(); it++)
      {
        Eigen::Vector2d vec = m_safety_area_border_points.row(it);
        geometry_msgs::Point pt;
        pt.x = vec.x();
        pt.y = vec.y();
        pt.z = 0.0;
        auto tfd = m_transformer.transformHeaderless(tf, pt);
        if (!tfd.has_value())
        {
          ROS_ERROR("Safety area could not be transformed!");
          return;
        } else
        {
          m_safety_area_border_points.row(it).x() = tfd.value().x;
          m_safety_area_border_points.row(it).y() = tfd.value().y;
        }
      }
    }

    //}

    /* transform polygon_obstacle_points //{ */

    for (auto& mat : m_safety_area_polygon_obstacle_points)
    {
      for (int it = 0; it < mat.rows(); it++)
      {
        assert(mat.cols() == 3);
        Eigen::Vector3d vec = mat.row(it);
        geometry_msgs::Point pt;
        pt.x = vec.x();
        pt.y = vec.y();
        pt.z = vec.z();
        auto tfd = m_transformer.transformHeaderless(tf, pt);
        if (!tfd.has_value())
        {
          ROS_ERROR("Safety area could not be transformed!");
          return;
        } else
        {
          mat.row(it).x() = tfd.value().x;
          mat.row(it).y() = tfd.value().y;
          mat.row(it).z() = tfd.value().z;
        }
      }
    }

    //}

    /* transform point_obstacle_points //{ */

    for (auto& mat : m_safety_area_point_obstacle_points)
    {
      for (int it = 0; it < mat.rows(); it++)
      {
        assert(mat.cols() == 3);
        Eigen::Vector3d vec = mat.row(it);
        geometry_msgs::Point pt;
        pt.x = vec.x();
        pt.y = vec.y();
        pt.z = vec.z();
        auto tfd = m_transformer.transformHeaderless(tf, pt);
        if (!tfd.has_value())
        {
          ROS_ERROR("Safety area could not be transformed!");
          return;
        } else
        {
          mat.row(it).x() = tfd.value().x;
          mat.row(it).y() = tfd.value().y;
          mat.row(it).z() = tfd.value().z;
        }
      }
    }

    //}

    try
    {
      m_safety_area =
          std::make_shared<mrs_lib::SafetyZone>(m_safety_area_border_points, m_safety_area_polygon_obstacle_points, m_safety_area_point_obstacle_points);
      ROS_INFO("[%s]: Safety zone intialized!", m_node_name.c_str());
    }
    catch (mrs_lib::SafetyZone::BorderError)
    {
      ROS_ERROR("[%s]: Exception caught. Wrong configruation for the safety zone border polygon.", m_node_name.c_str());
      return;
    }
    catch (mrs_lib::SafetyZone::PolygonObstacleError)
    {
      ROS_ERROR("[%s]: Exception caught. Wrong configuration for one of the safety zone polygon obstacles.", m_node_name.c_str());
      return;
    }
    catch (mrs_lib::SafetyZone::PointObstacleError)
    {
      ROS_ERROR("[%s]: Exception caught. Wrong configuration for one of the safety zone point obstacles.", m_node_name.c_str());
      return;
    }
    m_safety_area_init_timer.stop();
    m_is_initialized = true;
    m_safety_area_initialized = true;
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
    {
      std::scoped_lock lck(m_ukf_mtx);
      auto ret = m_ukf.predict(ukf_estimate, u, Q, dt);
      ret.x(ukf::x::yaw) = mrs_lib::normalize_angle(ret.x(ukf::x::yaw), -M_PI, M_PI);
      return ret;
    }
  }
  //}

  /* update_ukf_estimate() method //{ */
  void BalloonFilter::update_ukf_estimate(const pose_cov_t& measurement, const ros::Time& stamp, const theta_t& plane_theta)
  {
    auto [ukf_estimate_exists, ukf_estimate, ukf_last_update, ukf_n_updates] =
        mrs_lib::get_mutexed(m_ukf_estimate_mtx, m_ukf_estimate_exists, m_ukf_estimate, m_ukf_last_update, m_ukf_n_updates);

    const auto pos = measurement.pos_cov.pos;
    const double dt = (stamp - ukf_last_update).toSec();
    if (dt < 0.0)
      return;

    ukf_estimate = predict_ukf_estimate(ukf_estimate, dt, plane_theta, ball_speed_at_time(ukf_last_update));
    std::optional<double> yaw_opt = std::nullopt;
    if (measurement.ori_cov.has_value())
    {
      ori_cov_t ori_cov = measurement.ori_cov.value();
      // get the velocity direction of the measurement
      const pos_t xvec3d = ori_cov.quat*pos_t::UnitX();
      // rotation from the rheiv plane to the to the XY plane
      const quat_t rot_world_to_plane = quat_t::FromTwoVectors(plane_theta.block<3, 1>(0, 0), pos_t::UnitZ());
      // rotate the velocity vector to get its plane coordinates (it's just a vector, so no translation is required)
      const pos_t xvec2d = rot_world_to_plane*xvec3d;
      // get the yaw of the velocity vector in the plane coordinates
      yaw_opt = std::atan2(xvec2d.y(), xvec2d.x());
    }

    {
      std::scoped_lock lck(m_ukf_mtx);
      if (yaw_opt.has_value())
      {
        const double yaw = yaw_opt.value();
        const double est_yaw = ukf_estimate.x(ukf::x::yaw);
        const double yaw_diff = mrs_lib::normalize_angle(yaw - est_yaw, -M_PI, M_PI);
        const double n_yaw = est_yaw + yaw_diff;
        UKF::z_t z(4);
        z << pos.x(), pos.y(), pos.z(), n_yaw;
        UKF::R_t R = UKF::R_t::Zero(4, 4);
        R.block<3, 3>(0, 0) = measurement.pos_cov.cov;
        R(3, 3) = measurement.ori_cov.value().ypr_cov(0, 0);

        m_ukf.setObservationModel(ukf::obs_model_f_pose);
        ukf_estimate = m_ukf.correct(ukf_estimate, z, R);
        ukf_estimate.x(ukf::x::yaw) = mrs_lib::normalize_angle(ukf_estimate.x(ukf::x::yaw), -M_PI, M_PI);
        ROS_INFO_THROTTLE(0, "[UKF]: Updating current estimate using point [%.2f, %.2f, %.2f] and yaw %.2f", pos.x(), pos.y(),
                          pos.z(), n_yaw);
      }
      else
      {
        UKF::z_t z = pos;
        UKF::R_t R = measurement.pos_cov.cov;

        m_ukf.setObservationModel(ukf::obs_model_f_pos);
        ukf_estimate = m_ukf.correct(ukf_estimate, z, R);
        ukf_estimate.x(ukf::x::yaw) = mrs_lib::normalize_angle(ukf_estimate.x(ukf::x::yaw), -M_PI, M_PI);
        ROS_INFO_THROTTLE(0, "[UKF]: Updating current estimate using point [%.2f, %.2f, %.2f]", pos.x(), pos.y(),
                          pos.z());
      }
    }
    ukf_last_update = stamp;
    ukf_n_updates++;
    ukf_estimate_exists = true;

    set_mutexed(m_ukf_estimate_mtx, std::make_tuple(ukf_estimate_exists, ukf_estimate, ukf_last_update, ukf_n_updates),
                std::forward_as_tuple(m_ukf_estimate_exists, m_ukf_estimate, m_ukf_last_update, m_ukf_n_updates));
  }
  //}

  /* update_ukf_estimate() method //{ */

  void add_vec_marker(pos_t orig, pos_t dir, visualization_msgs::MarkerArray& to_msg, std_msgs::Header header, pos_t color, int id)
  {
    visualization_msgs::Marker msg;
    msg.header = header;
    msg.id = id;
    msg.pose.orientation.w = 1.0;
    msg.color.a = 1;
    msg.color.r = color.x();
    msg.color.g = color.y();
    msg.color.b = color.z();
    msg.scale.x = 0.1;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    msg.type = visualization_msgs::Marker::ARROW;
    msg.pose.position.x = orig.x();
    msg.pose.position.y = orig.y();
    msg.pose.position.z = orig.z();
    geometry_msgs::Point pt;
    msg.points.push_back(pt);
    pt.x = dir.x();
    pt.y = dir.y();
    pt.z = dir.z();
    msg.points.push_back(pt);
    to_msg.markers.push_back(msg);
  }

  // This function doesn't increase the counters or stamps of the UKF!
  void BalloonFilter::update_ukf_estimate(const circle3d_t& circle, const theta_t& rheiv_plane)
  {
    auto ukf_estimate = mrs_lib::get_mutexed(m_ukf_estimate_mtx, m_ukf_estimate);

    visualization_msgs::MarkerArray msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = m_world_frame_id;

    int id = 0;
    add_vec_marker({0,0,0}, circle.center, msg, header, {1,0,0}, id++);
    // rotation FROM world TO circle plane
    quat_t rot_to_circ_coords = quat_t::FromTwoVectors(circle.normal.cast<double>(), pos_t::UnitZ());

    const pos_t cur_pos = get_pos(ukf_estimate.x);
    add_vec_marker({0,0,0}, cur_pos, msg, header, {0,0.5,0}, id++);

    const pos_t diff_vec = cur_pos - circle.center;
    add_vec_marker(circle.center, diff_vec, msg, header, {1,1,0}, id++);

    const pos_t pos_circ_coords = rot_to_circ_coords*diff_vec;
    add_vec_marker({0,0,0}, pos_circ_coords , msg, header, {0,1,0}, id++);

    const double circ_dist = std::abs(pos_circ_coords .norm() - circle.radius);
    UKF::z_t z(1);
    UKF::R_t R(1, 1);

    // orientation of the rheiv-fitted plane (in which we want the curvature oriented)
    const quat_t rheiv_quat = plane_orientation(rheiv_plane);
    // if the planes have different orientation, the curvature needs to be flipped
    const bool plane_flip = anax_t(quat_t::FromTwoVectors(rheiv_quat*pos_t::UnitZ(), rot_to_circ_coords.inverse()*pos_t::UnitZ())).angle() < M_PI_2 ? false : true;
    double yaw = ukf_estimate.x(ukf::x::yaw);
    if (plane_flip)
    {
      ROS_ERROR("[]: FLIPPING");
      yaw = yaw + M_PI;
    }
    const pos_t yaw_vec = 3.0*pos_t(cos(yaw), sin(yaw), 0);
    add_vec_marker(pos_circ_coords, yaw_vec, msg, header, {0,0,1}, id++);

    const pos_t yaw_vec_glob = rot_to_circ_coords.inverse()*yaw_vec;
    add_vec_marker(circle.center, yaw_vec_glob, msg, header, {0,0,0.5}, id++);

    add_vec_marker(circle.center, circle.normal, msg, header, {0.5,0.5,0.5}, id++);

    const pos_t yaw_normal = pos_circ_coords.cross(yaw_vec);
    add_vec_marker({0,0,0}, yaw_normal, msg, header, {1,1,1}, id++);
    const double curv_sign = yaw_normal.z() > 0 ? 1.0 : -1.0;

    m_pub_dbg.publish(msg);

    // check if point lies on the circle
    if (circ_dist < m_circle_snap_threshold_distance)
    {

      // if the yaw in the first quadrant is positive, curvature is also positive
      z(0) = curv_sign/circle.radius; // but don't forget to compensate potential flip between rheiv and circle planes
      // the measurement noise when the measurement is associated to the circle is a parameter
      R(0) = m_ukf_meas_std_curv_circ;
      ROS_INFO_THROTTLE(MSG_THROTTLE, "[UKF]: Updating current estimate using circle radius %.2fm to curvature %.2f", circle.radius, z(0));
    }
    else
    {
      // if the point does not lie on the circle, assume it's on the line
      // and set the curvature accordingly
      z(0) = 0.0;
      // the measurement noise when the measurement is not associated to the circle is a different parameter
      R(0) = m_ukf_meas_std_curv_line;
      ROS_INFO_THROTTLE(MSG_THROTTLE, "[UKF]: Updating current estimate using line");
    }

    {
      std::scoped_lock lck(m_ukf_mtx);
      m_ukf.setObservationModel(ukf::obs_model_f_curv);
      ukf_estimate = m_ukf.correct(ukf_estimate, z, R);
      ukf_estimate.x(ukf::x::yaw) = ukf_estimate.x(ukf::x::yaw);
    }

    set_mutexed(m_ukf_estimate_mtx, std::make_tuple(ukf_estimate),
                std::forward_as_tuple(m_ukf_estimate));
  }
  //}

  /* estimate_ukf_initial_state() method //{ */
  std::optional<UKF::statecov_t> BalloonFilter::estimate_ukf_initial_state(const theta_t& plane_theta)
  {
    const auto [rheiv_pts, rheiv_covs, rheiv_stamps] = mrs_lib::get_mutexed(m_rheiv_data_mtx, m_rheiv_pts, m_rheiv_covs, m_rheiv_stamps);

    assert(rheiv_pts.size() == rheiv_covs.size());
    assert(rheiv_pts.size() == rheiv_stamps.size());
    assert(!rheiv_pts.empty());
    const int n_pts = rheiv_pts.size();

    ros::Time cur_time = ros::Time::now();
    ros::Time last_stamp;
    std::vector<std::tuple<pos_t, cov_t, ros::Time>> used_meass;
    for (int it = n_pts - 1; it >= 0; it--)
    {
      const auto cur_pt = rheiv_pts.at(it);
      const auto cur_cov = rheiv_covs.at(it);
      const auto cur_stamp = rheiv_stamps.at(it);
      if (it == n_pts - 1)
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
      ROS_ERROR("[BalloonFilter]: No recent points available for initial state estimation. Newest point is %.2fs old, need at most %.2fs.",
                (cur_time - rheiv_stamps.back()).toSec(), m_ukf_init_history_duration.toSec());
      return std::nullopt;
    }
    ROS_INFO("[UKF]: Trying to initialize UKF using %lu/%i points.", used_meass.size(), n_pts);

    UKF::statecov_t statecov;
    ros::Time prev_stamp;
    bool statecov_initd = false;

    for (int it = used_meass.size() - 1; it >= 0; it--)
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
      } else
      {
        const double dt = (cur_stamp - prev_stamp).toSec();
        /* assert(dt >= 0.0); */
        if (dt < 0.0)
          continue;

        statecov = predict_ukf_estimate(statecov, dt, plane_theta, ball_speed_at_time(prev_stamp));
        {
          std::scoped_lock lck(m_ukf_mtx);
          m_ukf.setObservationModel(ukf::obs_model_f_pos);
          statecov = m_ukf.correct(statecov, cur_pt, cur_cov);
          statecov.x(ukf::x::yaw) = mrs_lib::normalize_angle(statecov.x(ukf::x::yaw), -M_PI, M_PI);
        }
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
      ROS_INFO("[UKF]: Initializing estimate using point [%.2f, %.2f, %.2f], yaw %.2f and curvature %.2f", init_statecov.x(ukf::x::x),
               init_statecov.x(ukf::x::y), init_statecov.x(ukf::x::z), init_statecov.x(ukf::x::yaw), init_statecov.x(ukf::x::c));

      set_mutexed(m_ukf_estimate_mtx, std::make_tuple(init_statecov, true, stamp, 1),
                  std::forward_as_tuple(m_ukf_estimate, m_ukf_estimate_exists, m_ukf_last_update, m_ukf_n_updates));
    }
  }
  //}

  /* reset_ukf_estimate() method //{ */
  void BalloonFilter::reset_ukf_estimate()
  {
    set_mutexed(m_ukf_estimate_mtx, std::make_tuple(false, ros::Time::now(), 0),
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
      A << 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0, 0, 1, 0, 0, 1 * dt, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 1 * dt, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1 * dt, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    else
      A << 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    const LKF::Q_t Q = dt * LKF::Q_t(m_lkf_process_std.asDiagonal()).block(0, 0, m_lkf_n_states, m_lkf_n_states);
    const LKF::u_t u;
    m_lkf.A = A;
    const auto ret = m_lkf.predict(lkf_estimate, u, Q, dt);
    return ret;
  }
  //}

  /* update_lkf_estimate() method //{ */
  void BalloonFilter::update_lkf_estimate(const pose_cov_t& measurement, const ros::Time& stamp)
  {
    auto [lkf_estimate_exists, lkf_estimate, lkf_last_update, lkf_n_updates] =
        mrs_lib::get_mutexed(m_lkf_estimate_mtx, m_lkf_estimate_exists, m_lkf_estimate, m_lkf_last_update, m_lkf_n_updates);

    const auto pos = measurement.pos_cov.pos;
    ROS_INFO_THROTTLE(MSG_THROTTLE, "[LKF]: Updating current estimate using point [%.2f, %.2f, %.2f]", pos.x(), pos.y(),
                      pos.z());
    const double dt = (stamp - lkf_last_update).toSec();
    if (dt < 0.0)
      return;

    lkf_estimate = predict_lkf_estimate(lkf_estimate, dt);
    std::optional<pos_t> vel_opt = std::nullopt;
    if (measurement.ori_cov.has_value())
    {
      ori_cov_t ori_cov = measurement.ori_cov.value();
      const pos_t xvec3d = ori_cov.quat*pos_t::UnitX();
      vel_opt = xvec3d * ball_speed_at_time(stamp);
    }

    {
      std::scoped_lock lck(m_ukf_mtx);
      if (vel_opt.has_value())
      {
        pos_t vel = vel_opt.value();
        LKF::z_t z(6);
        z << pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z();
        UKF::R_t R = UKF::R_t::Zero(6, 6);
        R.block<3, 3>(0, 0) = measurement.pos_cov.cov;
        R.block<3, 3>(3, 3) = measurement.ori_cov.value().ypr_cov;

        m_lkf.H = LKF::H_t::Identity(6, m_lkf_n_states);
        lkf_estimate = m_lkf.correct(lkf_estimate, z, R);
        ROS_INFO_THROTTLE(MSG_THROTTLE, "[LKF]: Updating current estimate using point [%.2f, %.2f, %.2f] and velocity [%.2f, %.2f, %.2f]", pos.x(), pos.y(),
                          pos.z(), vel.x(), vel.y(), vel.z());
      }
      else
      {
        LKF::z_t z = pos;
        LKF::R_t R = measurement.pos_cov.cov;

        m_lkf.H = LKF::H_t::Identity(3, m_lkf_n_states);
        lkf_estimate = m_lkf.correct(lkf_estimate, z, R);
        ROS_INFO_THROTTLE(MSG_THROTTLE, "[lKF]: Updating current estimate using point [%.2f, %.2f, %.2f]", pos.x(), pos.y(),
                          pos.z());
      }
    }
    lkf_last_update = stamp;
    lkf_n_updates++;
    lkf_estimate_exists = true;

    set_mutexed(m_lkf_estimate_mtx, std::make_tuple(lkf_estimate_exists, lkf_estimate, lkf_last_update, lkf_n_updates),
                std::forward_as_tuple(m_lkf_estimate_exists, m_lkf_estimate, m_lkf_last_update, m_lkf_n_updates));
  }
  //}

  /* estimate_lkf_initial_state() method //{ */
  std::optional<LKF::statecov_t> BalloonFilter::estimate_lkf_initial_state()
  {
    const auto [rheiv_pts, rheiv_covs, rheiv_stamps] = mrs_lib::get_mutexed(m_rheiv_data_mtx, m_rheiv_pts, m_rheiv_covs, m_rheiv_stamps);

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
    for (int it = n_pts - 1; it >= 0; it--)
    {
      const auto cur_pt = rheiv_pts.at(it);
      const auto cur_cov = rheiv_covs.at(it);
      const auto cur_stamp = rheiv_stamps.at(it);
      if (it == n_pts - 1)
        last_stamp = cur_stamp;
      if (cur_time - cur_stamp > m_lkf_init_history_duration)
        break;
      used_meass.push_back({cur_pt, cur_cov, cur_stamp});
    }

    if (used_meass.empty())
    {
      ROS_ERROR("[BalloonFilter]: No recent points available for initial state estimation. Newest point is %.2fs old, need at most %.2fs.",
                (cur_time - rheiv_stamps.back()).toSec(), m_lkf_init_history_duration.toSec());
      return std::nullopt;
    }
    ROS_INFO("[LKF]: Trying to initialize lkf using %lu/%i points.", used_meass.size(), n_pts);

    LKF::statecov_t statecov;
    ros::Time prev_stamp;
    bool statecov_initd = false;
    bool speed_initd = false;

    for (int it = used_meass.size() - 1; it >= 0; it--)
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
      } else
      {
        const double dt = (cur_stamp - prev_stamp).toSec();
        if (dt < 0.0)
          continue;
        if (!speed_initd)
        {
          const pos_t prev_pos = get_pos(statecov.x);
          const pos_t cur_pos = cur_pt;
          const pos_t cur_spd = (cur_pos - prev_pos) / dt;
          statecov.x.block<3, 1>(3, 0) = cur_spd;
          speed_initd = true;
        }
        statecov = predict_lkf_estimate(statecov, dt);

        m_lkf.H = LKF::H_t::Identity(3, m_lkf_n_states);
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
        ROS_INFO(
            "[LKF]: Initializing estimate using point [%.2f, %.2f, %.2f], velocity [%.2f, %.2f, %.2f], acceleration [%.2f, %.2f, %.2f]\n speed: %.2f\n accel: "
            "%.2f",
            init_statecov.x(lkf::x::x), init_statecov.x(lkf::x::y), init_statecov.x(lkf::x::z), init_statecov.x(lkf::x::dx), init_statecov.x(lkf::x::dy),
            init_statecov.x(lkf::x::dz), init_statecov.x(lkf::x::ddx), init_statecov.x(lkf::x::ddy), init_statecov.x(lkf::x::ddz),
            init_statecov.x.block<3, 1>(3, 0).norm(), init_statecov.x.block<3, 1>(6, 0).norm());
      else
        ROS_INFO("[LKF]: Initializing estimate using point [%.2f, %.2f, %.2f], velocity [%.2f, %.2f, %.2f]\n speed: %.2f", init_statecov.x(lkf::x::x),
                 init_statecov.x(lkf::x::y), init_statecov.x(lkf::x::z), init_statecov.x(lkf::x::dx), init_statecov.x(lkf::x::dy), init_statecov.x(lkf::x::dz),
                 init_statecov.x.block<3, 1>(3, 0).norm());

      set_mutexed(m_lkf_estimate_mtx, std::make_tuple(init_statecov, true, stamp, 1),
                  std::forward_as_tuple(m_lkf_estimate, m_lkf_estimate_exists, m_lkf_last_update, m_lkf_n_updates));
    }
  }
  //}

  /* reset_lkf_estimate() method //{ */
  void BalloonFilter::reset_lkf_estimate()
  {
    set_mutexed(m_lkf_estimate_mtx, std::make_tuple(false, ros::Time::now(), 0),
                std::forward_as_tuple(m_lkf_estimate_exists, m_lkf_last_update, m_lkf_n_updates));
    ROS_WARN("[%s]: LKF estimate ==RESET==.", m_node_name.c_str());
  }
  //}

  /* predict_lkf_states() method //{ */
  std::vector<std::pair<LKF::x_t, ros::Time>> BalloonFilter::predict_lkf_states(const LKF::statecov_t initial_statecov, const ros::Time& initial_timestamp,
                                                                                const double prediction_horizon, const double prediction_step,
                                                                                const double set_speed = std::numeric_limits<double>::quiet_NaN())
  {
    assert(prediction_step > 0.0);
    assert(prediction_horizon > 0.0);
    const int n_pts = round(prediction_horizon / prediction_step);

    LKF::statecov_t statecov = initial_statecov;
    if (!std::isnan(set_speed))
    {
      const double cur_speed = statecov.x.block<3, 1>(3, 0).norm();
      statecov.x.block<3, 1>(3, 0) *= set_speed / cur_speed;
      statecov.P.block<3, 3>(3, 3) *= set_speed / cur_speed;
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
  // |               Circle fitting related methods               |
  // --------------------------------------------------------------

  /* reset_circle_estimate() method //{ */
  void BalloonFilter::reset_circle_estimate()
  {
    std::scoped_lock lck(m_circle_mtx);
    m_circle_valid = false;

    ROS_WARN("[%s]: Circle estimate ==RESET==.", m_node_name.c_str());
  }
  //}

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  /* reset_estimates() method //{ */
  void BalloonFilter::reset_estimates()
  {
    reset_lkf_estimate();
    reset_ukf_estimate();
    reset_rheiv_estimate();
    reset_circle_estimate();
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

  visualization_msgs::MarkerArray BalloonFilter::circle_visualization(const circle3d_t& circle, const std_msgs::Header& header)
  {
    visualization_msgs::MarkerArray ret;
    const quat_t quat = quat_t::FromTwoVectors(pos_t::UnitZ(), circle.normal.cast<double>());
    const auto radius = circle.radius;

    /* lines (the circle itself) //{ */

    {
      visualization_msgs::Marker lines;
      lines.header = header;
      lines.type = visualization_msgs::Marker::LINE_LIST;
      lines.color.a = 0.8;
      lines.color.b = 1.0;
      lines.scale.x = 0.1;
      lines.ns = "circle";
      lines.pose.orientation.w = quat.w();
      lines.pose.orientation.x = quat.x();
      lines.pose.orientation.y = quat.y();
      lines.pose.orientation.z = quat.z();
      lines.pose.position.x = circle.center.x();
      lines.pose.position.y = circle.center.y();
      lines.pose.position.z = circle.center.z();

      constexpr int circ_pts_per_meter_radius = 10;
      int circ_pts = std::round(radius * circ_pts_per_meter_radius);
      if (circ_pts % 2)
        circ_pts++;
      for (int it = 0; it < circ_pts; it++)
      {
        const float angle = M_PI / (circ_pts / 2.0f) * it;
        geometry_msgs::Point pt;
        pt.x = radius * cos(angle);
        pt.y = radius * sin(angle);
        pt.z = 0;
        lines.points.push_back(pt);
      }
      ret.markers.push_back(lines);
    }

    //}

    /* radisu text //{ */

    {
      Eigen::Vector3d edge = circle.center + quat * Eigen::Vector3d(radius, 0, 0);
      visualization_msgs::Marker text;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.header = header;
      text.color.a = 0.8;
      text.color.b = 1.0;
      text.scale.z = 1.0;
      text.text = "r:" + std::to_string(radius);
      text.ns = "radius text";
      text.pose.orientation.w = quat.w();
      text.pose.orientation.x = quat.x();
      text.pose.orientation.y = quat.y();
      text.pose.orientation.z = quat.z();
      text.pose.position.x = edge.x();
      text.pose.position.y = edge.y();
      text.pose.position.z = edge.z();
      ret.markers.push_back(text);
    }

    //}

    return ret;
  }

  /* balloon_filter::BallLocation //{ */
  balloon_filter::BallLocation BalloonFilter::to_output_message(const pos_cov_t& estimate, const std_msgs::Header& header)
  {
    balloon_filter::BallLocation ret;

    ret.header = header;
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
  geometry_msgs::PoseWithCovarianceStamped BalloonFilter::to_output_message2(const pos_cov_t& estimate, const std_msgs::Header& header)
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

  /* msg2cov() method //{ */
  cov_t BalloonFilter::msg2cov(const ros_cov_t& msg_cov, int offset)
  {
    cov_t cov;
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        cov(r, c) = msg_cov[(r+offset) * 6 + (c+offset)];
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
    const bool height_valid = pt.z() > m_bounds_z_min && pt.z() < m_bounds_z_max;
    const bool sane_values = !pt.array().isNaN().any() && !pt.array().isInf().any();
    const bool in_safety_area = !m_safety_area || m_safety_area->isPointValid(pt.x(), pt.y(), pt.z());
    return height_valid && sane_values && in_safety_area;
  }
  //}

  /* plane_orientation() method //{ */
  quat_t BalloonFilter::plane_orientation(const theta_t& plane_theta)
  {
    const quat_t ret = quat_t::FromTwoVectors(pos_t::UnitZ(), plane_theta.block<3, 1>(0, 0));
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
    m_bounds_z_min = cfg.bounds__z__min;
    m_bounds_z_max = cfg.bounds__z__max;
    /* m_meas_filt_desired_dt = ros::Duration(cfg.meas_filt__desired_dt); */
    /* m_meas_filt_loglikelihood_threshold = cfg.meas_filt__loglikelihood_threshold; */
    /* m_meas_filt_covariance_inflation = cfg.meas_filt__covariance_inflation; */
    m_max_time_since_update = cfg.max_time_since_update;
    m_min_updates_to_confirm = cfg.min_updates_to_confirm;
    m_circle_min_radius = cfg.circle__min_radius;

    /* UKF-related //{ */

    m_ukf_prediction_horizon = cfg.ukf__prediction_horizon;
    m_ukf_prediction_step = cfg.ukf__prediction_step;

    m_ukf_process_std(ukf::x::x) = m_ukf_process_std(ukf::x::y) = m_ukf_process_std(ukf::x::z) = cfg.ukf__process_std__position;
    m_ukf_process_std(ukf::x::yaw) = cfg.ukf__process_std__yaw;
    /* m_ukf_process_std(ukf::x_s) = cfg.process_std__speed; */
    m_ukf_process_std(ukf::x::c) = cfg.ukf__process_std__curvature;

    m_ukf_init_std(ukf::x::yaw) = cfg.ukf__init_std__yaw;
    /* m_init_std(ukf::x_s) = cfg.init_std__speed; */
    m_ukf_init_std(ukf::x::c) = cfg.ukf__init_std__curvature;
    m_ukf_meas_std_curv_circ = cfg.ukf__meas_std__curvature_circ;
    m_ukf_meas_std_curv_line = cfg.ukf__meas_std__curvature_line;

    //}

    /* LKF-related //{ */

    m_lkf_min_init_points = cfg.lkf__min_init_points;
    m_lkf_prediction_horizon = cfg.lkf__prediction_horizon;
    m_lkf_prediction_step = cfg.lkf__prediction_step;
    m_lkf_max_speed_err = cfg.lkf__max_speed_err;

    m_lkf_process_std(lkf::x::x) = m_lkf_process_std(lkf::x::y) = m_lkf_process_std(lkf::x::z) = cfg.lkf__process_std__position;
    m_lkf_process_std(lkf::x::dx) = m_lkf_process_std(lkf::x::dy) = m_lkf_process_std(lkf::x::dz) = cfg.lkf__process_std__velocity;
    if (m_lkf_use_acceleration)
      m_lkf_process_std(lkf::x::ddx) = m_lkf_process_std(lkf::x::ddy) = m_lkf_process_std(lkf::x::ddz) = cfg.lkf__process_std__acceleration;

    m_lkf_init_std(lkf::x::dx) = m_lkf_init_std(lkf::x::dy) = m_lkf_init_std(lkf::x::dz) = cfg.lkf__init_std__velocity;
    if (m_lkf_use_acceleration)
      m_lkf_init_std(lkf::x::ddx) = m_lkf_init_std(lkf::x::ddy) = m_lkf_init_std(lkf::x::ddz) = cfg.lkf__init_std__acceleration;

    //}

    m_rheiv_min_pts = cfg.rheiv__min_points;
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
    const auto uav_name = pl.load_param2<std::string>("uav_name");

    /* initialize transformer //{ */

    m_transformer = mrs_lib::Transformer(m_node_name, uav_name);

    //}

    /* const int measurements_buffer_length = pl.load_param2<int>("meas_filt/buffer_length"); */

    const double processing_period = pl.load_param2<double>("processing_period");
    const double prediction_period = pl.load_param2<double>("prediction_period");
    pl.load_param("world_frame_id", m_world_frame_id);
    pl.load_param("uav_frame_id", m_uav_frame_id);
    pl.load_param("ball_speed1", m_ball_speed1);
    pl.load_param("ball_speed2", m_ball_speed2);
    pl.load_param("ball_wire_length", m_ball_wire_length);

    pl.load_param("circle/max_radius", m_circle_max_radius);
    pl.load_param("circle/fit_threshold_distance", m_circle_fit_threshold_distance);
    pl.load_param("circle/snap_threshold_distance", m_circle_snap_threshold_distance);

    const ros::Duration ball_speed_change_after = pl.load_param2("ball_speed_change_after");
    // TODO: set the time in some smarter manner
    m_ball_speed_change = ros::Time::now() + ball_speed_change_after;
    pl.load_param("rheiv/fitting_period", m_rheiv_fitting_period);
    pl.load_param("rheiv/max_line_points_ratio", m_rheiv_max_line_pts_ratio);
    pl.load_param("rheiv/max_points", m_rheiv_max_pts);
    pl.load_param("rheiv/visualization_size", m_rheiv_visualization_size);
    const double rheiv_timeout_s = pl.load_param2<double>("rheiv/timeout");

    pl.load_param("ukf/init_history_duration", m_ukf_init_history_duration);

    pl.load_param("lkf/use_acceleration", m_lkf_use_acceleration);
    pl.load_param("lkf/init_history_duration", m_lkf_init_history_duration);

    // | ----------------------- safety area ---------------------- |
    /*  //{ */

    const auto use_safety_area = pl.load_param2<bool>("safety_area/use_safety_area");
    m_safety_area_frame = pl.load_param2<std::string>("safety_area/frame_name");

    if (use_safety_area)
    {
      m_safety_area_border_points = pl.load_matrix_dynamic2("safety_area/safety_area", -1, 2);

      const auto obstacle_polygons_enabled = pl.load_param2<bool>("safety_area/polygon_obstacles/enabled");
      if (obstacle_polygons_enabled)
        m_safety_area_polygon_obstacle_points = pl.load_matrix_array2("safety_area/polygon_obstacles", std::vector<Eigen::MatrixXd>{});

      const auto obstacle_points_enabled = pl.load_param2<bool>("safety_area/point_obstacles/enabled");
      if (obstacle_points_enabled)
        m_safety_area_point_obstacle_points = pl.load_matrix_array2("safety_area/point_obstacles", std::vector<Eigen::MatrixXd>{});

      // TODO: remove this when param loader supports proper loading
      for (auto& matrix : m_safety_area_polygon_obstacle_points)
        matrix.transposeInPlace();

      m_safety_area_init_timer = nh.createTimer(ros::Duration(1.0), &BalloonFilter::init_safety_area, this);
    } else
    {
      m_safety_area_initialized = true;
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
    m_sh_localized = smgr.create_handler<geometry_msgs::PoseWithCovarianceStamped, time_consistent>("localized_ball", ros::Duration(5.0));

    m_reset_estimates_server = nh.advertiseService("reset_estimates", &BalloonFilter::reset_estimates_callback, this);
    //}

    /* publishers //{ */

    m_pub_dbg = nh.advertise<visualization_msgs::MarkerArray>("dbg_marker", 1);
    m_pub_pcl_dbg = nh.advertise<sensor_msgs::PointCloud2>("dbg_pcl", 1);
    m_pub_chosen_meas = nh.advertise<balloon_filter::BallLocation>("chosen_measurement", 1);
    m_pub_chosen_meas_dbg = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("chosen_measurement_dbg", 1);

    m_pub_plane_dbg = nh.advertise<visualization_msgs::MarkerArray>("fitted_plane_marker", 1);
    m_pub_plane_dbg2 = nh.advertise<geometry_msgs::PoseStamped>("fitted_plane_pose", 1);
    m_pub_used_pts = nh.advertise<sensor_msgs::PointCloud2>("fit_points", 1);
    m_pub_fitted_plane = nh.advertise<balloon_filter::PlaneStamped>("fitted_plane", 1);

    m_pub_ball_prediction = nh.advertise<balloon_filter::BallPrediction>("ball_prediction", 1);
    m_pub_pred_path_dbg = nh.advertise<nav_msgs::Path>("predicted_path", 1);
    m_pub_circle_dbg = nh.advertise<visualization_msgs::MarkerArray>("fitted_circle_marker", 1);

    //}

    /* profiler //{ */

    m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

    //}

    /* initialize UKF //{ */
    {

      UKF::transition_model_t tra_model(ukf::tra_model_f);
      UKF::observation_model_t obs_model(ukf::obs_model_f_pos);
      m_ukf = UKF(ukf::tra_model_f, ukf::obs_model_f_pos);
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
      const LKF::H_t H = LKF::H_t::Identity(3, m_lkf_n_states);
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
      const std::chrono::duration<double> rheiv_timeout(rheiv_timeout_s * 1000);
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

    /* m_prev_measurements.set_capacity(measurements_buffer_length); */
    reset_estimates();
    if (m_safety_area_initialized)
      m_is_initialized = true;

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(processing_period), &BalloonFilter::main_loop, this);
    m_rheiv_loop_timer = nh.createTimer(ros::Duration(m_rheiv_fitting_period), &BalloonFilter::rheiv_loop, this);
    m_prediction_loop_timer = nh.createTimer(ros::Duration(prediction_period), &BalloonFilter::prediction_loop, this);

    //}

    ROS_INFO("[%s]: onInit complete", m_node_name.c_str());
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
