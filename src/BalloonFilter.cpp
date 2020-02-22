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

    if (m_lkf_estimate_exists)
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
      return;
    }

    if (!rheiv_new_data)
      return;

    ros::Time stamp = ros::Time::now();
    bool success = false;
    rheiv::theta_t theta;
    if (rheiv_pts.size() >= (size_t)m_rheiv_min_pts)
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

      /* Eigen::VectorXf line_params; */
      /* double line_pts_ratio; */
      if (pointcloud->size() > (size_t)m_rheiv_min_pts) // this should be redundant, but it still crashes...
      {
        auto model_l = boost::make_shared<pcl::SampleConsensusModelLine<pt_XYZ_t>>(pointcloud);
        pcl::RandomSampleConsensus<pt_XYZ_t> fitter(model_l);
        fitter.setDistanceThreshold(m_rheiv_line_threshold_distance);
        fitter.computeModel();
        /* fitter.getModelCoefficients(line_params); */
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

      std_msgs::Header header;
      header.frame_id = m_world_frame_id;
      header.stamp = stamp;

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
      // otherwise display the line fit to the user if no previous fit is available to be used
      /* else if (!m_rheiv_theta_valid) */
      /* { */
      /*   if (m_pub_plane_dbg.getNumSubscribers() > 0) */
      /*   { */
      /*     std::string txt = "ratio: " + std::to_string(line_pts_ratio); */
      /*     line3d_t line; */
      /*     line.origin = line_params.block<3, 1>(0, 0).cast<double>(); */
      /*     line.direction = 100.0*line_params.block<3, 1>(3, 0).cast<double>(); */
      /*     line.origin -= line.direction/2.0; */
      /*     line.radius = m_linefit_threshold_distance; */
      /*     m_pub_plane_dbg.publish(line_visualization(line, header, txt)); */
      /*   } */
      /* } */

      if (success)
      {
        /* try to publish plane debug visualization markers //{ */

        if (m_pub_plane_dbg.getNumSubscribers() > 0 || m_pub_plane_dbg2.getNumSubscribers() > 0)
        {
          if (m_pub_plane_dbg.getNumSubscribers() > 0)
            m_pub_plane_dbg.publish(plane_visualization(theta, header));
          if (m_pub_plane_dbg2.getNumSubscribers() > 0)
            m_pub_plane_dbg2.publish(to_output_message2(theta, header));
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

  /* linefit_loop() method //{ */
  void BalloonFilter::linefit_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    if (!m_is_initialized)
      return;

    const auto [plane_theta_valid, plane_theta] = get_mutexed(m_rheiv_theta_mtx, m_rheiv_theta_valid, m_rheiv_theta);
    if (!plane_theta_valid)
      return;

    // threadsafe copy the data to be fitted with the lines
    const auto linefit_pose_stampeds = get_mutexed(m_linefit_data_mtx, m_linefit_pose_stampeds);

    ros::Time stamp = ros::Time::now();
    bool success = false;
    rheiv::theta_t theta;
    if (linefit_pose_stampeds.size() > (size_t)m_linefit_min_pts)
    {
      const ros::WallTime start_t = ros::WallTime::now();
      const vec3_t plane_normal = plane_theta.block<3, 1>(0, 0);
      const float  plane_d = -plane_theta(3);
      const quat_t plane_quat = quat_t::FromTwoVectors(plane_normal, vec3_t::UnitZ());
      const quat_t plane_iquat = plane_quat.inverse();
      const vec3_t plane_offset_pt = plane_normal * plane_d / (plane_normal.dot(plane_normal));

      const auto plane_iquat_f = plane_iquat.cast<float>();
      const auto plane_offset_pt_f = plane_offset_pt.cast<float>();

      std_msgs::Header header;
      header.frame_id = m_world_frame_id;
      header.stamp = stamp;

      const float oldest_stamp = (stamp - m_start_time - m_linefit_max_pt_age).toSec();
      pc_XYZt_t::Ptr pointcloud = boost::make_shared<pc_XYZt_t>();
      pointcloud->reserve(linefit_pose_stampeds.size());
      pcl_conversions::toPCL(header, pointcloud->header);
      for (auto pose : linefit_pose_stampeds)
      {
        // do not use too old points
        if (pose.stamp < oldest_stamp)
          continue;
        // transform the point to the plane
        const auto pt = plane_quat * (pose.pose.pos - plane_offset_pt);
        // add the point to the pointcloud
        pt_XYZt_t pclpt;
        pclpt.x = pt.x();
        pclpt.y = pt.y();
        /* pclpt.z = pt.z(); */
        pclpt.z = 0.0;
        pclpt.intensity = pose.stamp;
        pointcloud->push_back(pclpt);
      }

      // | ------------------- Fit the first line ------------------- |

      // note that the resulting values will already be transformed back to world
      pcl::PointIndicesPtr line1_inliers = boost::make_shared<pcl::PointIndices>();
      pc_XYZt_t::Ptr line1_points = boost::make_shared<pc_XYZt_t>();
      line3d_t line1;
      if (pointcloud->size() > (size_t)m_linefit_min_pts) // this should be redundant, but it still crashes...
      /*  //{ */

      {
        Eigen::VectorXf line1_params;
        auto model_l = boost::make_shared<pcl::SampleConsensusModelLine<pt_XYZt_t>>(pointcloud);
        pcl::RandomSampleConsensus<pt_XYZt_t> fitter(model_l);
        fitter.setDistanceThreshold(m_linefit_threshold_distance);
        fitter.computeModel();
        fitter.getModelCoefficients(line1_params);
        fitter.getInliers(line1_inliers->indices);

        // extract inliers of line1
        pcl::ExtractIndices<pt_XYZt_t> ei;
        ei.setInputCloud(pointcloud);
        ei.setIndices(line1_inliers);
        ei.filter(*line1_points);

        line1.origin = line1_params.block<3, 1>(0, 0).cast<double>();
        line1.direction = line1_params.block<3, 1>(3, 0).cast<double>();
        line1.radius = 0.1;
      }

      //}
      else
      {
        ROS_WARN_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Not enough valid points to fit a line (%lu is less than %d). Aborting.", pointcloud->size(), m_linefit_min_pts);
        return;
      }

      // | ------------------- Fit the second line ------------------ |

      // note that the resulting values will already be transformed back to world
      pcl::PointIndicesPtr line2_inliers = boost::make_shared<pcl::PointIndices>();
      pc_XYZt_t::Ptr line2_points = boost::make_shared<pc_XYZt_t>();
      line3d_t line2;
      /*  //{ */

      {
        Eigen::VectorXf line2_params;
        std::vector<uint8_t> line1_outlier_flags(pointcloud->size(), 1);
        for (const auto l1in : line1_inliers->indices)
          line1_outlier_flags.at(l1in) = 0;
        // line1_outlier_flags[i] is 1 if pointcloud[i] is outlier, 0 else

        pcl::PointIndicesPtr line1_outlier_inds = boost::make_shared<pcl::PointIndices>();
        line1_outlier_inds->indices.reserve(pointcloud->size() - line1_inliers->indices.size());
        for (int it = 0; (size_t)it < pointcloud->size(); it++)
          if (line1_outlier_flags.at(it))
            line1_outlier_inds->indices.push_back(it);
        // line1_outlier_inds now contains indices of line1 outliers

        // extract points which do not lie on line1
        pc_XYZt_t::Ptr line1_outliers = boost::make_shared<pc_XYZt_t>();
        pcl::ExtractIndices<pt_XYZt_t> ei;
        ei.setInputCloud(pointcloud);
        ei.setIndices(line1_outlier_inds);
        ei.filter(*line1_outliers);

        if (line1_outliers->size() >= (size_t)m_linefit_min_pts)
        {
          // fit a line to the remaining points
          auto model_l = boost::make_shared<pcl::SampleConsensusModelLine<pt_XYZt_t>>(line1_outliers);
          pcl::RandomSampleConsensus<pt_XYZt_t> fitter(model_l);
          fitter.setDistanceThreshold(m_linefit_threshold_distance);
          fitter.computeModel();
          fitter.getModelCoefficients(line2_params);
          fitter.getInliers(line2_inliers->indices);

          // extract inliers of line2
          ei.setInputCloud(line1_outliers);
          ei.setIndices(line2_inliers);
          ei.filter(*line2_points);

          line2.origin = line2_params.block<3, 1>(0, 0).cast<double>();
          line2.direction = line2_params.block<3, 1>(3, 0).cast<double>();
          line2.radius = 0.1;
        }
        else
        {
          ROS_WARN_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Not enough valid points remaining to fit the second line (%lu is less than %d). Aborting.", line1_outliers->size(), m_linefit_min_pts);
          return;
        }
      }

      //}

      line3d_t chosen_line;
      // only continue if both lines got enough inliers
      if (line1_points->size() >= (size_t)m_linefit_min_pts && line2_points->size() >= (size_t)m_linefit_min_pts)
      {
        success = true;

        vec3_t line1_vel;
        {
          // sort the inliers by time
          sort_pcl(line1_points);
          const float line1_ori = estimate_line_orientation(line1_points, line1);
          line1_vel = line1_ori*line1.direction.normalized()*ball_speed_at_time(stamp);
          line1.direction = line1_vel;
        }

        vec3_t line2_vel;
        {
          // sort the inliers by time
          sort_pcl(line2_points);
          const float line2_ori = estimate_line_orientation(line2_points, line2);
          line2_vel = line2_ori*line2.direction.normalized()*ball_speed_at_time(stamp);
          line2.direction = line2_vel;
        }

        // check that the line point in different directions
        if (line1.direction.dot(line2.direction) > 0.0)
        {
          ROS_WARN_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Angle between fitted lines is too small (%.2f < %.2f)! Invalid.", std::acos(line1.direction.normalized().dot(line2.direction.normalized())), M_PI_2);
          // angle between the lines is less than pi/2
          success = false;
        }

        line1 = constrain_line_to_pts(line1, line1_points);
        line2 = constrain_line_to_pts(line2, line2_points);

        // transform the inliers back to 3D
        transform_pcl(line1_points, plane_iquat_f, plane_offset_pt_f);
        transform_pcl(line2_points, plane_iquat_f, plane_offset_pt_f);

        transform_line(line1, plane_iquat, plane_offset_pt);
        transform_line(line2, plane_iquat, plane_offset_pt);

        const auto [linefit_valid, prev_linefit] = get_mutexed(m_linefit_mtx, m_linefit_valid, m_linefit);
        if (linefit_valid)
        {
          const vec3_t prev_dir = prev_linefit.direction.normalized();
          const vec3_t line1_dir = line1.direction.normalized();
          const vec3_t line2_dir = line2.direction.normalized();
          // find the line with the more similar direction
          const double line1_cos = prev_dir.dot(line1_dir);
          const double line2_cos = prev_dir.dot(line2_dir);
          if (line1_cos > line2_cos)
            chosen_line = line1;
          else
            chosen_line = line2;
          const double upd_ang = std::acos(std::clamp(prev_dir.dot(chosen_line.direction.normalized()), -1.0, 0.0));
          if (std::isnan(upd_ang))
            ROS_INFO_THROTTLE(MSG_THROTTLE, "POSRALO SE TO");
          ROS_INFO_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Updating chosen line with a new fit (angle: %.2f).", upd_ang);
        }
        else
        {
          // chose the line with more inliers
          if (line1_points->size() > line2_points->size())
            chosen_line = line1;
          else
            chosen_line = line2;
          ROS_INFO_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Initializing chosen line with a new fit (length: %.2fm).", chosen_line.direction.norm());
        }
      }
      else
      {
        ROS_WARN_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Lines do not have enough inliers to continue (%lu or %lu is less than %d).", line1_points->size(), line2_points->size(), m_linefit_min_pts);
        success = false;
      }

      const ros::WallTime end_t = ros::WallTime::now();
      const double fit_dur = (end_t - start_t).toSec();
      if (success)
      {
        set_mutexed(m_linefit_mtx, std::make_tuple(true, chosen_line),
                    std::forward_as_tuple(m_linefit_valid, m_linefit));

        const auto res_pose = back_up_line(chosen_line, m_linefit_back_up);
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = header;
        pose_msg.pose = res_pose;
        m_pub_line_endpose.publish(pose_msg);

        {
          std::string txt = "LINE1";
          if (line1 == chosen_line)
            txt = "CHOSEN LINE";
          m_pub_line1.publish(line_visualization(line1, header, txt));
          m_pub_line1_pts.publish(line1_points);
        }

        {
          std::string txt = "LINE2";
          if (line2 == chosen_line)
            txt = "CHOSEN LINE";
          m_pub_line2.publish(line_visualization(line2, header, txt));
          m_pub_line2_pts.publish(line2_points);
        }
        ROS_INFO_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Fitted lines with %lu and %lu points and %.2fm and %.2fm lengths in %.3fs.", line1_points->size(), line2_points->size(), line1.direction.norm(), line2.direction.norm(), fit_dur);
      }
      else
      {
        ROS_WARN_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Failed to fit lines (took %.2fs)!", fit_dur);
      }

    } else  // if (rheiv_pts.size() > (size_t)m_rheiv_min_pts)
    {
      // Still waiting for enough points to fit the plane through.
      ROS_WARN_STREAM_THROTTLE(MSG_THROTTLE, "[LINEFIT]: Not enough points to fit line (" << linefit_pose_stampeds.size() << "/ " << m_linefit_min_pts << ").");
    }
  }
  //}

  /* prediction_loop() method //{ */
  void BalloonFilter::prediction_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    if (!m_is_initialized)
      return;

    std_msgs::Float64 msg; msg.data = ball_speed_at_time(ros::Time::now());
    m_pub_gt_ball_speed.publish(msg);

    const auto [lkf_estimate_exists, lkf_estimate, lkf_last_update, lkf_n_updates] =
        mrs_lib::get_mutexed(m_lkf_estimate_mtx, m_lkf_estimate_exists, m_lkf_estimate, m_lkf_last_update, m_lkf_n_updates);

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

    const auto [plane_theta_valid, plane_theta] = mrs_lib::get_mutexed(m_rheiv_theta_mtx, m_rheiv_theta_valid, m_rheiv_theta);
    if (plane_theta_valid)
    {
      fitted_plane = to_output_message(plane_theta);
      fitted_plane.valid = true;
    }

    filter_state.fitted_plane = fitted_plane;
    message.filter_state = filter_state;
    message.predicted_path = predicted_path;

    m_pub_pred_path_dbg.publish(predicted_path);
    m_pub_prediction.publish(message);
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


    pos_t pos;
    pos.x() = msg.pose.pose.position.x;
    pos.y() = msg.pose.pose.position.y;
    pos.z() = msg.pose.pose.position.z;

    pose_t det_pose;
    det_pose.pos = pos;

    pose_cov_t det_pose_cov;
    det_pose_cov.pos_cov.pos = pos;
    det_pose_cov.pos_cov.cov = msg2cov(msg.pose.covariance);

    // check if the direction covariance is sufficiently low to use it
    if (msg.pose.covariance.at(6*6-1) < M_PI_2)
    {
      quat_t quat;
      quat.w() = msg.pose.pose.orientation.w;
      quat.x() = msg.pose.pose.orientation.x;
      quat.y() = msg.pose.pose.orientation.y;
      quat.z() = msg.pose.pose.orientation.z;

      det_pose.quat = quat;

      const cov_t ypr_cov = msg2cov(msg.pose.covariance, 3);
      det_pose_cov.ori_cov = {quat, ypr_cov};
    }

    add_linefit_data(det_pose, msg.header.stamp);
    add_rheiv_data(det_pose_cov.pos_cov.pos, det_pose_cov.pos_cov.cov, msg.header.stamp);

    /* // copy the latest plane fit */
    /* const auto [plane_theta_valid, plane_theta] = get_mutexed(m_rheiv_theta_mtx, m_rheiv_theta_valid, m_rheiv_theta); */

    /* if (plane_theta_valid) */
    /*   const auto [line1, line2] = fit_lines(m_poses, plane_theta); */

    /* update the LKF //{ */

    if (m_lkf_estimate_exists)
    {
      update_lkf_estimate(det_pose_cov, msg.header.stamp);
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
      if (vel_opt.has_value())
      {
        pos_t vel = vel_opt.value();
        LKF::z_t z(6);
        z << pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z();
        LKF::R_t R = LKF::R_t::Zero(6, 6);
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

    const auto [plane_theta_valid, plane_theta] = mrs_lib::get_mutexed(m_rheiv_theta_mtx, m_rheiv_theta_valid, m_rheiv_theta);
    const auto [linefit_valid, linefit] = mrs_lib::get_mutexed(m_linefit_mtx, m_linefit_valid, m_linefit);

    LKF::statecov_t statecov = initial_statecov;
    if (!std::isnan(set_speed))
    {
      const double cur_speed = statecov.x.block<3, 1>(3, 0).norm();
      statecov.x.block<3, 1>(3, 0) *= set_speed / cur_speed;
      statecov.P.block<3, 3>(3, 3) *= set_speed / cur_speed;
    }

    // snap the velocity prediction to the line direction
    if (linefit_valid)
    {
      const pos_t cur_pos = statecov.x.block<3, 1>(0, 0);
      const vec3_t line_dir = linefit.direction.normalized();
      const float cur_pos_proj_dist = line_dir.dot(cur_pos - linefit.origin);
      const pos_t cur_pos_proj = linefit.origin + cur_pos_proj_dist*line_dir;
      const double line_pos_dist = (cur_pos_proj - cur_pos).norm();

      const vec3_t cur_vel_dir = statecov.x.block<3, 1>(3, 0).normalized();
      const double line_vel_ang = std::acos(line_dir.dot(cur_vel_dir));
      if (line_pos_dist < m_linefit_snap_dist && std::abs(line_vel_ang) < m_linefit_snap_ang)
      {
        statecov.x.block<3, 1>(3, 0) = set_speed * line_dir.normalized();
        ROS_INFO_THROTTLE(MSG_THROTTLE, "[LKF]: Snapping velocity to the line (dist: %.2fm, ang: %.2f).", line_pos_dist, line_vel_ang);
      }
      else
      {
        ROS_INFO_THROTTLE(MSG_THROTTLE, "[LKF]: NOT snapping velocity to the line - too far (dist: %.2fm > %.2fm or ang: %.2f > %.2f)!", line_pos_dist, m_linefit_snap_dist, line_vel_ang, m_linefit_snap_ang);
      }
    }

    // snap (project) the position to the plane
    if (plane_theta_valid)
    {
      const vec3_t plane_normal = plane_theta.block<3, 1>(0, 0);
      const float  plane_d = -plane_theta(3);
      const quat_t plane_quat = quat_t::FromTwoVectors(plane_normal, vec3_t::UnitZ());
      const quat_t plane_iquat = plane_quat.inverse();
      const vec3_t plane_offset_pt = plane_normal * plane_d / (plane_normal.dot(plane_normal));

      const pos_t cur_pos = statecov.x.block<3, 1>(0, 0);
      pos_t cur_pos_proj = plane_quat * (cur_pos - plane_offset_pt);
      const float plane_pos_dist = cur_pos_proj.z();
      if (plane_pos_dist < m_rheiv_snap_dist)
      {
        cur_pos_proj.z() = 0.0;
        cur_pos_proj = plane_iquat*cur_pos_proj + plane_offset_pt;
        statecov.x.block<3, 1>(0, 0) = cur_pos_proj;
        ROS_INFO_THROTTLE(MSG_THROTTLE, "[LKF]: Snapping position to the plane (dist: %.2fm)", plane_pos_dist);
      }
      else
      {
        ROS_INFO_THROTTLE(MSG_THROTTLE, "[LKF]: NOT snapping position to the line - too far (dist: %.2fm > %.2fm)!", plane_pos_dist, m_rheiv_snap_dist);
      }
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
  // |                Line fitting related methods                |
  // --------------------------------------------------------------

  void BalloonFilter::transform_pcl(pc_XYZt_t::Ptr pcl, const Eigen::Quaternionf& quat, const Eigen::Vector3f trans)
  {
    for (auto& pt : pcl->points)
      pt.getVector3fMap() = quat * pt.getVector3fMap() + trans;
  }

  void BalloonFilter::transform_line(line3d_t& line, const quat_t& quat, const vec3_t& trans)
  {
    line.origin = quat*line.origin + trans;
    line.direction = quat*line.direction;
  }

  void BalloonFilter::sort_pcl(pc_XYZt_t::Ptr pcl)
  {
    std::sort(std::begin(pcl->points), std::end(pcl->points),
              // comparison lambda function
              [](const pt_XYZt_t& a, const pt_XYZt_t& b) { return a.intensity < b.intensity; });
  }

  pos_t BalloonFilter::to_eigen(const pt_XYZt_t& pt)
  {
    return {pt.x, pt.y, pt.z};
  }

  float BalloonFilter::estimate_line_orientation(pc_XYZt_t::Ptr points, const line3d_t& line)
  {
    const vec3_t line_dir = line.direction;
    std::vector<float> orientations;
    orientations.reserve(points->size());
    pos_t prev_pt = to_eigen(points->points.front());
    for (int it = 1; (size_t)it < points->size(); it++)
    {
      const pos_t cur_pt = to_eigen(points->at(it));
      const vec3_t cur_vel = cur_pt - prev_pt;
      const float cur_ori = cur_vel.dot(line_dir);
      orientations.push_back(cur_ori);
      prev_pt = cur_pt;
    }
    auto median_it = std::begin(orientations) + orientations.size()/2;
    std::nth_element(std::begin(orientations), median_it , std::end(orientations));
    return mrs_lib::sign(*median_it);
  }

  line3d_t BalloonFilter::constrain_line_to_pts(const line3d_t& line, pc_XYZt_t::Ptr points)
  {
    std::vector<float> dists;
    dists.reserve(points->size());
    const vec3_t line_dir = line.direction.normalized();
    for (const auto& pt : points->points)
    {
      const pos_t cur_pt = to_eigen(pt);
      const float cur_dist = line_dir.dot(cur_pt - line.origin);
      dists.push_back(cur_dist);
    }
    const auto [mindist_it, maxdist_it] = std::minmax_element(std::begin(dists), std::end(dists));
    const pos_t start = line.origin + (*mindist_it)*line_dir;
    const pos_t end = line.origin + (*maxdist_it)*line_dir;
    const vec3_t dir = end-start;
    line3d_t ret = line;
    ret.origin = start;
    ret.direction = dir;
    return ret;
  }

  geometry_msgs::Pose BalloonFilter::back_up_line(const line3d_t& line, const double amount)
  {
    geometry_msgs::Pose ret;

    const double clamped_amount = std::clamp(amount, 0.0, line.direction.norm()/2.0);
    const vec3_t backup_vec = -clamped_amount*line.direction.normalized();
    const pos_t backed_up_pos = line.origin + line.direction + backup_vec;
    const quat_t quat = quat_t::FromTwoVectors(vec3_t::UnitX(), line.direction);

    ret.position.x = backed_up_pos.x();
    ret.position.y = backed_up_pos.y();
    ret.position.z = backed_up_pos.z();

    ret.orientation.w = quat.w();
    ret.orientation.x = quat.x();
    ret.orientation.y = quat.y();
    ret.orientation.z = quat.z();

    return ret;
  }

  /* reset_line_estimate() method //{ */
  void BalloonFilter::reset_line_estimate()
  {
    std::scoped_lock lck(m_linefit_mtx);
    m_linefit_valid = false;

    ROS_WARN("[%s]: Line estimate ==RESET==.", m_node_name.c_str());
  }
  //}

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  /* reset_estimates() method //{ */
  void BalloonFilter::reset_estimates()
  {
    reset_lkf_estimate();
    reset_rheiv_estimate();
    reset_line_estimate();
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

  /* line_visualization() //{ */
  
  visualization_msgs::MarkerArray BalloonFilter::line_visualization(const line3d_t& line, const std_msgs::Header& header, const std::string& text = "")
  {
    visualization_msgs::MarkerArray ret;
    Eigen::Vector3d center = line.origin + line.direction/2.0;
  
    /* lines (the line itself) //{ */
  
    {
      visualization_msgs::Marker linem;
      linem.id = 2;
      linem.header = header;
      linem.type = visualization_msgs::Marker::ARROW;
      linem.color.a = 0.8;
      linem.color.b = 1.0;
      linem.scale.x = 0.1;
      linem.scale.y = 0.5;
      linem.scale.z = 1.0;
      linem.ns = "line";
      linem.pose.orientation.w = 1.0;
      geometry_msgs::Point pt;
      pt.x = line.origin.x();
      pt.y = line.origin.y();
      pt.z = line.origin.z();
      linem.points.push_back(pt);
      pt.x += line.direction.x();
      pt.y += line.direction.y();
      pt.z += line.direction.z();
      linem.points.push_back(pt);
      ret.markers.push_back(linem);
    }
  
    //}
  
    /* text //{ */
  
    if (!text.empty())
    {
      visualization_msgs::Marker textm;
      textm.id = 3;
      textm.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      textm.header = header;
      textm.color.a = 0.8;
      textm.color.b = 1.0;
      textm.scale.z = 1.0;
      textm.text = text;
      textm.ns = "radius text";
      textm.pose.orientation.w = 1.0;
      textm.pose.position.x = center.x();
      textm.pose.position.y = center.y();
      textm.pose.position.z = center.z();
      ret.markers.push_back(textm);
    }
  
    //}
  
    return ret;
  }
  
  //}

  /* plane_visualization //{ */
  visualization_msgs::MarkerArray BalloonFilter::plane_visualization(const theta_t& plane_theta, const std_msgs::Header& header)
  {
    visualization_msgs::MarkerArray ret;
  
    /* delete line markers //{ */
  
    {
      visualization_msgs::Marker m;
      m.action = visualization_msgs::Marker::DELETE;
      m.id = 3;
      m.ns = "line";
      ret.markers.push_back(m);
      m.id = 4;
      m.ns = "radius text";
      ret.markers.push_back(m);
    }
  
    //}

    const vec3_t plane_normal = plane_theta.block<3, 1>(0, 0);
    const float  plane_d = -plane_theta(3);
    const quat_t quat = quat_t::FromTwoVectors(vec3_t::UnitZ(), plane_normal);
    const vec3_t pos = plane_normal * plane_d / (plane_normal.dot(plane_normal));

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

  /* geometry_msgs::PoseStamped //{ */
  geometry_msgs::PoseStamped BalloonFilter::to_output_message2(const theta_t& plane_theta, const std_msgs::Header& header)
  {
    geometry_msgs::PoseStamped ret;

    const vec3_t plane_normal = plane_theta.block<3, 1>(0, 0);
    const float  plane_d = -plane_theta(3);
    const quat_t quat = quat_t::FromTwoVectors(vec3_t::UnitZ(), plane_normal);
    const vec3_t pos = plane_normal * plane_d / (plane_normal.dot(plane_normal));

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

    pl.load_param("linefit/threshold_distance", m_linefit_threshold_distance);
    pl.load_param("linefit/fitting_period", m_linefit_fitting_period);
    pl.load_param("linefit/min_points", m_linefit_min_pts);
    pl.load_param("linefit/back_up", m_linefit_back_up);
    pl.load_param("linefit/snap_distance", m_linefit_snap_dist);
    pl.load_param("linefit/snap_angle", m_linefit_snap_ang);
    pl.load_param("linefit/max_point_age", m_linefit_max_pt_age);

    pl.load_param("circle/min_radius", m_circle_min_radius);
    pl.load_param("circle/max_radius", m_circle_max_radius);

    const ros::Duration ball_speed_change_after = pl.load_param2("ball_speed_change_after");
    // TODO: set the time in some smarter manner
    m_ball_speed_change = ros::Time::now() + ball_speed_change_after;
    pl.load_param("rheiv/fitting_period", m_rheiv_fitting_period);
    pl.load_param("rheiv/line_threshold_distance", m_rheiv_line_threshold_distance);
    pl.load_param("rheiv/max_line_points_ratio", m_rheiv_max_line_pts_ratio);
    pl.load_param("rheiv/max_points", m_rheiv_max_pts);
    pl.load_param("rheiv/visualization_size", m_rheiv_visualization_size);
    pl.load_param("rheiv/snap_distance", m_rheiv_snap_dist);
    const double rheiv_timeout_s = pl.load_param2<double>("rheiv/timeout");

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

    m_pub_line_endpose = nh.advertise<geometry_msgs::PoseStamped>("line_endpose", 1);
    
    m_pub_chosen_meas = nh.advertise<balloon_filter::BallLocation>("chosen_measurement", 1);
    m_pub_chosen_meas_dbg = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("chosen_measurement_dbg", 1);

    m_pub_line1 = nh.advertise<visualization_msgs::MarkerArray>("fitted_line1_marker", 1);
    m_pub_line1_pts = nh.advertise<sensor_msgs::PointCloud2>("fitted_line1_points", 1);
    m_pub_line2 = nh.advertise<visualization_msgs::MarkerArray>("fitted_line2_marker", 1);
    m_pub_line2_pts = nh.advertise<sensor_msgs::PointCloud2>("fitted_line2_points", 1);

    m_pub_plane_dbg = nh.advertise<visualization_msgs::MarkerArray>("fitted_plane_marker", 1);
    m_pub_plane_dbg2 = nh.advertise<geometry_msgs::PoseStamped>("fitted_plane_pose", 1);
    m_pub_used_pts = nh.advertise<sensor_msgs::PointCloud2>("fit_points", 1);
    m_pub_fitted_plane = nh.advertise<balloon_filter::PlaneStamped>("fitted_plane", 1);

    m_pub_prediction = nh.advertise<balloon_filter::BallPrediction>("prediction", 1);
    m_pub_pred_path_dbg = nh.advertise<nav_msgs::Path>("predicted_path", 1);

    m_pub_gt_ball_speed = nh.advertise<std_msgs::Float64>("gt_ball_speed", 1);

    //}

    /* profiler //{ */

    m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

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

    /* initialize line fitting //{ */
    
    {
      /* m_linefit_new_data = false; */
      /* m_prev_measurements.set_capacity(measurements_buffer_length); */
      m_linefit_valid = false;
      constexpr int mission_duration = 15; // minutes
      constexpr int sensor_rate = 10;      // Hz
      m_linefit_pose_stampeds.reserve(mission_duration*60*sensor_rate);
    }
    
    //}

    reset_estimates();
    m_start_time = ros::Time::now();
    if (m_safety_area_initialized)
      m_is_initialized = true;

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(processing_period), &BalloonFilter::main_loop, this);
    m_rheiv_loop_timer = nh.createTimer(ros::Duration(m_rheiv_fitting_period), &BalloonFilter::rheiv_loop, this);
    m_linefit_loop_timer = nh.createTimer(ros::Duration(m_linefit_fitting_period), &BalloonFilter::linefit_loop, this);
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
