#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>
#include <ardrone_autonomy/Navdata.h>

#include <visnav2013_exercise/trajectory_visualizer.h>

/**
 * ARDroneOdometry, computes the position of the AR.Drone from its measurements.
 *
 * Extend the ARDroneOdometry::onNavdata(...) method with your solution.
 * Feel free to adapt the code to your needs ;)
 */
class ARDroneOdometry {
private:
	ros::Subscriber navdata_subscriber_;
	visnav2013_exercise::TrajectoryVisualizer visualizer_;

	tf::Transform pose_;
	float time_pre;
public:
	ARDroneOdometry(ros::NodeHandle& nh) :
			visualizer_(nh) {
		pose_.setIdentity();
		// subscribe to the '/ardrone/navdata' topic
		navdata_subscriber_ = nh.subscribe<ardrone_autonomy::Navdata>(
				"/ardrone/navdata", 1,
				boost::bind(&ARDroneOdometry::onNavdata, this, _1));

		ROS_INFO("Subscribed to '/ardrone/navdata' topic.");
	}

	/**
	 * Subscriber method, called for every received ardrone_autonomy::Navdata message.
	 */
	void onNavdata(const ardrone_autonomy::NavdataConstPtr& navdata) {
		ROS_INFO_STREAM("received navdata @" << navdata->header.stamp);


		ROS_INFO_STREAM(
				"vx: "<<navdata->vx << " mm/s"<<"; vy: "<<navdata->vy << " mm/s"<<"; yaw: "<<navdata->rotZ <<" Degree" << "; height: "<<navdata->altd <<" mm ");


		// TODO: compute odometry and update 'pose_' accordingly, use your solution from Exercise 1
		//Calculate the global velocity vector v_g from the local one v_l
		tfScalar roll=navdata->rotX, vx_l=navdata->vx,vy_l=navdata->vy,vz_l=navdata->vz;
		tfScalar pitch=navdata->rotY;
		tfScalar yaw=navdata->rotZ;
		tf::Vector3 v_l(vx_l,vy_l,vz_l),v_g;
		tf::Matrix3x3 rot;
		rot.setEulerYPR(yaw,pitch,roll);
		v_g = rot * v_l;

		//
		float t = navdata->header.stamp.sec + navdata->header.stamp.nsec *0.000000001;
		float dt = 0;
		if(time_pre){
			dt = t - time_pre;
		}
		time_pre = t;

		tfScalar x_t = pose_.getOrigin().getX()+v_g.getX()*dt;
		tfScalar y_t = pose_.getOrigin().getY()+v_g.getY()*dt;
		tfScalar z_t = navdata->altd;
		pose_.setOrigin(tf::Vector3(x_t,y_t,z_t));

		visualizer_.addPose(pose_).publish();

	}
};

int main(int argc, char **argv) {
	// initialize the ROS node
	ros::init(argc, argv, "Ex1ARDroneOdometry");

	// create a NodeHandle to subscribe/advertise topics
	ros::NodeHandle nh;

	// create our main class
	ARDroneOdometry odometry(nh);

	// wait until shutdown, e.g., until someone presses Ctrl+C
	ros::spin();

	return 0;
}
