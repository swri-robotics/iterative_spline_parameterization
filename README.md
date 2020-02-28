# iterative_spline_parameterization
This is a modified version of [MoveIt's iterative spline parameterization trajectory processing](https://github.com/ros-planning/moveit/blob/master/moveit_core/trajectory_processing/src/iterative_spline_parameterization.cpp) tool that removes dependencies on libraries specific to ROS and MoveIt. It still uses the Eigen library to represent vectors, although this probably is not technically required and could be simplified in the future.

[Ken Anderson](http://kennethanderson.github.io/index.html) is the original author of the iterative spline parameterization algorithm.

## Extremely Basic Usage Example

```
void do_parameterization(const trajectory_msgs::msg::JointTrajectory& traj_in,
                         trajectory_msgs::msg::JointTrajectory &traj_out)
{
  std::vector<iterative_spline_parameterization::TrajectoryState> waypoints;
  
  for (auto point : traj_in.points)
  {
    // Convert joint angles in message (vector of doubles) to Eigen MatrixXd
    Eigen::Matrix<double, 6, 1>  joint_angles(point.positions.data());
    
    waypoints.push_back(
      iterative_spline_parameterization::TrajectoryState(joint_angles,
                                                         Eigen::Matrix<double, 6, 1>::Zero(),
                                                         Eigen::Matrix<double, 6, 1>::Zero(),
                                                         0.0));
  }
  
  iterative_spline_parameterization::IterativeSplineParameterization isp(false);

  isp.computeTimeStamps(waypoints, 1.0, 1.0);
  
  for (auto point : waypoints)
  {
    // Convert joint angles from parameterization output (Eigen::MatrixXd)
    // to format in ROS message (vector of doubles)
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(point.time));

    pt.positions.resize(point.positions.size());
    Eigen::VectorXd::Map(&pt.positions[0], point.positions.size()) = point.positions;

    pt.velocities.resize(point.velocities.size());
    Eigen::VectorXd::Map(&pt.velocities[0], point.velocities.size()) = point.velocities;

    pt.accelerations.resize(point.accelerations.size());
    Eigen::VectorXd::Map(&pt.accelerations[0], point.accelerations.size()) = point.accelerations;

    traj_out.points.push_back(pt);
  }
}
```
