#ifndef TRAJECTORY_VISUALIZER_H_
#define TRAJECTORY_VISUALIZER_H_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

namespace visnav2013_exercise
{

class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer(ros::NodeHandle& nh);

  /**
   * Adds a pose at the current end of the trajectory.
   */
  TrajectoryVisualizer& addPose(const tf::Transform& pose);

  /**
   * Sends the internal MarkerArray message. Call this method to update the visualization,
   * if you added new poses through addPose(...).
   */
  void publish();
private:
  ros::Publisher marker_publisher_;
  visualization_msgs::MarkerArray markers_;
  visualization_msgs::Marker pose_marker_prototype_;
};

} /* namespace visnav2013_exercise */
#endif /* TRAJECTORY_VISUALIZER_H_ */
