// The following is a program  written to  estimate the pose of the DeepRacer
// relative to a given target.

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/TransformStamped.h>
#include <parallel_park/ServoCtrlMsg.h> // To replace the ServoCtrlMsg from DeepRacer's ctrl_pkg
//#include <cmath.h>

bool canPark;

//************************************************************
// Function to Compute the Distance Between Two Frames (Essentially 2 Tags)
float computeDistance(float xFirst, float yFirst, float zFirst, float xSecond, float ySecond, float zSecond){
	float xDistance = 0;
	float yDistance = 0;
	float zDistance = 0;

	xDistance = (xFirst - xSecond)*100;
	ROS_INFO_STREAM("\n\rxDistance: " << xDistance);

	return xDistance;
}

//************************************************************
// Callback function to register with tf2_ros::MessageFilter to be called when frame 3 is available
void frame3_Callback(const geometry_msgs::TransformStampedConstPtr& framePtr)
{
  int f;
  geometry_msgs::TransformStamped transform3;
  tf2_ros::Buffer tfBuffer3;
  tf2_ros::TransformListener tfListener3(tfBuffer3);

  try{
    transform3 = tfBuffer3.lookupTransform("tag_3", "cv_camera", ros::Time(0));
  }
  catch(tf2::TransformException &ex){
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

//************************************************************
// Main function
int main(int argc, char **argv){
	// Initialize the ROS system.
	ros::init(argc, argv, "parallel_park");

	// Establish this program as a ROS node
	ros::NodeHandle nh;

	// Send a log message to indicate the node is up and running
	ROS_INFO_STREAM("Parallel Park is up!");

	// Create a publisher object to publish to the /manual_drive topic to
	// control the servo
	ros::Publisher pub = nh.advertise<parallel_park::ServoCtrlMsg>("/manual_drive", 100);

	// Create buffer objects and transformListener objects to deal with
	// getting the transforms
	tf2_ros::Buffer tfBuffer1;
	tf2_ros::Buffer tfBuffer2;
  tf2_ros::Buffer tfBuffer3;
	tf2_ros::TransformListener tfListener1(tfBuffer1);
	tf2_ros::TransformListener tfListener2(tfBuffer2);
  tf2_ros::TransformListener tfListener3(tfBuffer3);

	// Set the sleep rate to 2s
	ros::Rate rate(2.0);

	// Perform the following as long as the node is running
	// while(ros::ok()){
	// 	// Objects to hold the Transforms
	// 	geometry_msgs::TransformStamped transformStamped1;
	// 	geometry_msgs::TransformStamped transformStamped2;
	// 	geometry_msgs::TransformStamped transformStamped3;
	//
	// 	// Try to get each transform, and warn if doesn't work each time
	// 	try{
	// 		transformStamped1 = tfBuffer1.lookupTransform("cv_camera", "tag_1", ros::Time(0));
	// 		transformStamped2 = tfBuffer2.lookupTransform("cv_camera", "tag_2", ros::Time(0));
	// 		//transformStamped3 = tfBuffer3.lookupTransform("cv_camera", "tag_3", ros::Time(0));
	// 	}
	// 	catch(tf2::TransformException &ex){
	// 		ROS_WARN("%s", ex.what());
	// 		ros::Duration(1.0).sleep();
	// 		continue;
	// 	}
	//
	// 	// Holders for each of the tags' frame translations
	// 	float x1 = transformStamped1.transform.translation.x;
	// 	float y1 = transformStamped1.transform.translation.y;
	// 	float z1 = transformStamped1.transform.translation.z;
	// 	float x2 = transformStamped2.transform.translation.x;
	// 	float y2 = transformStamped2.transform.translation.y;
	// 	float z2 = transformStamped2.transform.translation.z;
	// 	float x3 = transformStamped3.transform.translation.x;
	// 	float y3 = transformStamped3.transform.translation.y;

		// Create the single Servo Control Message that will be published
    // after computations
		parallel_park::ServoCtrlMsg control;

		// float xDistance = computeDistance(x1,y1,z1,x2,y2,z2);
		// float prevDistance = 0;
		//
		// // ROS_INFO_STREAM("Distance between tag 1 and tag 2: " << xDistance);
		// // If xDistance > 60cm, continue on to the next part
		// if(xDistance > 40){
			// control.angle = (float) (0.9);
			// control.throttle = (float) (0.65);
			// pub.publish(control);
			rate.sleep();

			control.angle = (float) (-0.9);
			control.throttle = (float) (-0.65);
			pub.publish(control);
			ROS_INFO_STREAM("Right turn");
			ros::Duration(0.95).sleep();

			control.angle = (float) (0.9);
			control.throttle = (float) (-0.65);
			pub.publish(control);
			ROS_INFO_STREAM("Left turn");
			ros::Duration(0.45).sleep();

			control.angle = (float) (-0.9);
			control.throttle = (float) (0.65);
			pub.publish(control);
			ROS_INFO_STREAM("Correct");
			ros::Duration(0.5).sleep();

			control.angle = (float) (0);
			control.throttle = (float) (0);
			ROS_INFO_STREAM("Rest");
			pub.publish(control);

			// prevDistance = xDistance;
		// }


      /*int tag3Ready = 1;
      ROS_INFO_STREAM("tag3Ready:" << tag3Ready);
      try{
        transformStamped3 = tfBuffer3.lookupTransform("tag_3", "cv_camera", ros::Time(0));
        tag3Ready = 1;
      }
      catch(tf2::TransformException &ex){
        ROS_WARN("tag_3 unavailable");
        ros::Duration(1.0).sleep();
        tag3Ready = 0;
      }
      float z3 = (transformStamped3.transform.translation.z)*100;
      ROS_INFO_STREAM("tag3Ready 2nd time:" << tag3Ready);
      ROS_INFO_STREAM("z3:" << z3);
      if(tag3Ready == 1 && z3 > 30)
      {
         ROS_INFO_STREAM("Here We Go");
         control.throttle = (float) (0.65);
         control.angle = (float) (0);
      }
      else{
        ROS_INFO_STREAM("No \t No \t No");
        control.throttle = (float) (0.0);
        control.angle = (float) (0);
      }*/
/*
    }
    else{
      canPark = false;
    }/*

		pub.publish(control);
	rate.sleep();
}

		/*
		  Algorithm to Park the car from beside tag 1 ((Taken from Ballinas et al.))
		    Givens: Steering angle:0.9 = 30 deg = pi/6,
		            wheelbase, L: 0.162m,
		            robot width, W: 0.193m,
		            rear axle to rear bumper, p: 0.035m,
		            initial points, (Xs, Ys), and
		            minimum distances (xmin,ymin)
		    Output: Whole path P
		    void pathPlanner(float angle, float L, float W, float p, float Xs, float Ys
		                    float xmin, float ymin)
		    {
		    // Calculate distance R
		    float R = L/(tan(angle));

		    // Find intersection point Yt
		    float Yt = R-(ymin+(W/2));

		    // Calculate aperture angle alpha
		    float alpha = angle; // Will test this, but I believe the turn angle is equal

		    // Calculate intersection point Xt
		    float Xt = R*cos(alpha);

		    // Calculate the minimum parking spot distance
		    Mmin = (2*Xt)+p-xmin;

		    // Calculate the goal position
		    float Xg = (Mmin/2) + xmin + (L/2) + p;
		    float Yg = R-(W+ymin);
		    float Yf = Yg;
		    float Xf = Mmin-p;
		    float P = (Xs,Ys)(Xt,Yt)+(Xt,Yt)(Xf,Yf)+(Xf,Yf)(Xg,Yg)

		  }
*/
	// }
}
