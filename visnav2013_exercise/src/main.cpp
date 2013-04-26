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

    // variables to use new pose estimation
	int time_pre_sec;
	int time_pre_nsec;
    tfScalar        roll,  pitch,  yaw;    // rotation angles
    tfScalar        vx_l,  vy_l,   vz_l;   // axis velocities
    tf::Vector3     v_g;                   // global velocity vectors
    tf::Matrix3x3   rot;                   // velocity rotation matrix
    tfScalar        x_t,    y_t,    z_t;   // current position
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

		//Print v_x, v_y, yaw, height and their units.
		ROS_INFO_STREAM(
				"vx: "<<navdata->vx << " mm/s"<<"; vy: "<<navdata->vy << " mm/s"<<"; yaw: "<<navdata->rotZ <<" Degree" << "; height: "<<navdata->altd <<" mm ");


		// TODO: compute odometry and update 'pose_' accordingly, use your solution from Exercise 1
		//Calculate the global velocity vector v_g from the local velocity vector v_l
        roll	=navdata->rotX;
        pitch	=navdata->rotY;
        yaw     =navdata->rotZ;
        vx_l	=navdata->vx;
        vy_l	=navdata->vy;
        vz_l	=navdata->vz;

        tf::Vector3 v_l(vx_l,vy_l,vz_l); // LOCAL VELOCITY VECTOR

		rot.setEulerZYX(yaw,pitch,roll);     
		v_g = rot * v_l;

		//Calculate the time difference in nano second.
		int t_sec = navdata->header.stamp.sec;
		int t_nsec = navdata->header.stamp.nsec;

		int dt = 0;
		if(time_pre_sec){
			dt = (t_sec - time_pre_sec)*1000000000 + (t_nsec - time_pre_nsec);
		}
		time_pre_sec = navdata->header.stamp.sec;
		time_pre_nsec = navdata->header.stamp.nsec;

//		ROS_INFO_STREAM("dt: "<<dt);

        x_t = pose_.getOrigin().getX()  +   v_g.getX()  *   dt;
        y_t = pose_.getOrigin().getY()  +   v_g.getY()  *   dt;
        z_t = navdata->altd;    // assignt altitude since z_t 0.0 in bag files
		pose_.setOrigin(tf::Vector3(x_t,y_t,z_t));
        pose_.setRotation(tf::Quaternion(yaw,pitch,roll));

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
