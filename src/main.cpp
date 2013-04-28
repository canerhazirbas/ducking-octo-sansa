#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>
#include <ardrone_autonomy/Navdata.h>
#include <math.h>

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

	// variables to use new pose estimation
	double time_pre_sec;
	double time_pre_nsec;
	tfScalar roll, pitch, yaw;    // rotation angles
	tfScalar vx_l, vy_l, vz_l;   // axis velocities
	tf::Vector3 v_g;                   // global velocity vectors
	tf::Matrix3x3 rot;                   // velocity rotation matrix
	tfScalar x_t, y_t, z_t;   // current position
	tfScalar dist;
	tfScalar heightSum;
	int iteration;
public:
	ARDroneOdometry(ros::NodeHandle& nh) :
			visualizer_(nh) {
		pose_.setIdentity();
		// subscribe to the '/ardrone/navdata' topic
		navdata_subscriber_ = nh.subscribe<ardrone_autonomy::Navdata>(
				"/ardrone/navdata", 1,
				boost::bind(&ARDroneOdometry::onNavdata, this, _1));

		ROS_INFO("Subscribed to '/ardrone/navdata' topic.");
		dist = 0;
		heightSum = 0;
		iteration = 0;
	}

	/**
	 * Subscriber method, called for every received ardrone_autonomy::Navdata message.
	 */
	void onNavdata(const ardrone_autonomy::NavdataConstPtr& navdata) {
		ROS_INFO_STREAM("received navdata @" << navdata->header.stamp);

		//print v_x, v_y, yaw, height and their units.
		ROS_INFO_STREAM(
				"vx: "<<navdata->vx << " mm/s"<<"; vy: "<<navdata->vy << " mm/s"<<"; yaw: "<<navdata->rotZ <<" Degree" << "; height: "<<navdata->altd <<" mm ");

		//calculate the global velocity vector v_g from the local velocity vector v_l
		roll = navdata->rotX*M_PI/180.0;
		pitch = navdata->rotY*M_PI/180.0;
		yaw = navdata->rotZ*M_PI/180.0;
		vx_l = navdata->vx;
		vy_l = navdata->vy;
		vz_l = navdata->vz;

		tf::Vector3 v_l(vx_l, vy_l, vz_l); // LOCAL VELOCITY VECTOR

		rot.setEulerYPR(yaw, pitch, roll);
		v_g = rot * v_l;

		ROS_INFO_STREAM("v_g_x: "<<v_g.getX()<<", v_g_y: "<<v_g.getY());

		//Calculate the time difference in nano second.
		double t_sec = navdata->header.stamp.sec;
		double t_nsec = navdata->header.stamp.nsec;

		double dt = 0;
		if (time_pre_sec) {
			dt = (t_sec - time_pre_sec)
					+ 0.0000000001 * (t_nsec - time_pre_nsec);
		}

		time_pre_sec = navdata->header.stamp.sec;
		time_pre_nsec = navdata->header.stamp.nsec;

		//calculate pose differences
		float dx = v_g.getX() * dt;
		float dy = v_g.getY() * dt;
		float dz = navdata->altd - pose_.getOrigin().getZ();

		x_t = pose_.getOrigin().getX() + dx;
		y_t = pose_.getOrigin().getY() + dy;
		z_t = navdata->altd;    // assign altitude since z_t 0.0 in bag files

		dist = dist + sqrt(dx*dx+dy*dy+dz*dz); //Calculate travelled distance
		heightSum = heightSum + navdata->altd;
		iteration = iteration + 1;
		ROS_INFO_STREAM("Distance travelled: " << dist << " Mean height: " << heightSum/iteration);

		//update pose
		tf::Quaternion quaternion;
		quaternion.setEuler(yaw, pitch, roll);
		pose_.setRotation(quaternion);
		pose_.setOrigin(tf::Vector3(x_t, y_t, z_t));

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
