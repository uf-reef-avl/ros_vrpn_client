/*
# Copyright (c) 2011, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
## author Chih-Hung Aaron King (Healthcare Robotics Lab, Georgia Tech.)
*/

//== This application listens for a rigid body named 'Tracker' on a remote machine
//== and publishes & tf it's position and orientation through ROS.


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>
#include <math.h>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

// Daman
//#include <LinearMath/btQuaternion.h>
#include <tf/LinearMath/Quaternion.h>
// /Daman

void VRPN_CALLBACK track_target (void *, const vrpn_TRACKERCB t);

class TargetState{
    public:
        geometry_msgs::TransformStamped target;
        geometry_msgs::PoseStamped target_pose_stamped;

        geometry_msgs::TransformStamped ned_target;
        geometry_msgs::PoseStamped ned_pose_stamped;
};


TargetState *target_state;
std::string frame_id;

// set to true in the VRPN callback function.
bool fresh_data = false;
vrpn_TRACKERCB prev_vrpn_data;


class Rigid_Body {
    private:
        ros::Publisher target_pub;
        ros::Publisher pose_stamp_pub;
        ros::Publisher ned_target_pub;
        ros::Publisher ned_pose_stamp_pub;
        tf::TransformBroadcaster br;
        vrpn_Connection *connection;
        vrpn_Tracker_Remote *tracker;

    public:
        Rigid_Body(ros::NodeHandle& nh, std::string server_ip,
                   int port)
        {
            target_pub = nh.advertise<geometry_msgs::TransformStamped>("nwu/pose", 100);
            pose_stamp_pub = nh.advertise<geometry_msgs::PoseStamped>("nwu/pose_stamped",100);

            ned_target_pub = nh.advertise<geometry_msgs::TransformStamped>("ned/pose", 100);
            ned_pose_stamp_pub = nh.advertise<geometry_msgs::PoseStamped>("ned/pose_stamped",100);

            std::string connec_nm = server_ip + ":" + boost::lexical_cast<std::string>(port);
            connection = vrpn_get_connection_by_name(connec_nm.c_str());
            std::string target_name = nh.getNamespace().substr(1);
            tracker = new vrpn_Tracker_Remote(target_name.c_str(), connection);
            this->tracker->register_change_handler(NULL, track_target);
        }

        void publish_target_state(TargetState *target_state)
        {
            br.sendTransform(target_state->target);
            target_pub.publish(target_state->target);
            pose_stamp_pub.publish(target_state->target_pose_stamped);

            ned_target_pub.publish(target_state->ned_target);
            ned_pose_stamp_pub.publish(target_state->ned_pose_stamped);
        }

        void step_vrpn()
        {
            this->tracker->mainloop();
            this->connection->mainloop();
        }
};

//== Tracker Position/Orientation Callback ==--
void VRPN_CALLBACK track_target (void *, const vrpn_TRACKERCB t)
{
    // Daman
	//btQuaternion q_orig(t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
	tf::Quaternion q_orig(t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
    //btQuaternion q_fix(0.70710678, 0., 0., 0.70710678);
	tf::Quaternion q_fix(0.70710678, 0., 0., 0.70710678);
	// /Daman

    // optitrak <-- funky <-- object
    // the q_fix.inverse() esures that when optitrak_funky says 0 0 0
    // for roll pitch yaw, there is still a rotation that aligns the
    // object frame with the /optitrak frame (and not /optitrak_funky)
    // Daman
	//btQuaternion q_rot = q_fix * q_orig * q_fix.inverse();
	tf::Quaternion q_rot = q_fix * q_orig * q_fix.inverse();
	// /Daman

    //btScalar ang = q_rot.getAngle();
    // Daman
	//btVector3 axis = q_rot.getAxis();
	tf::Vector3 axis = q_rot.getAxis();
    //btVector3 pos(t.pos[0], -t.pos[2], t.pos[1]);
	tf::Vector3 pos(t.pos[0], -t.pos[2], t.pos[1]);
    //btVector3 new_pos = pos.rotate(axis, ang);

    // verifying that each callback indeed gives fresh data.
    if ( prev_vrpn_data.quat[0] == t.quat[0] and \
         prev_vrpn_data.quat[1] == t.quat[1] and \
         prev_vrpn_data.quat[2] == t.quat[2] and \
         prev_vrpn_data.quat[3] == t.quat[3] and \
         prev_vrpn_data.pos[0] == t.pos[0] and \
         prev_vrpn_data.pos[1] == t.pos[1] and \
         prev_vrpn_data.pos[2] == t.pos[2] )
        ROS_WARN("Repeated Values");

    prev_vrpn_data = t;

    target_state->target.transform.translation.x = pos.x();
    //target_state->target.transform.translation.x = pos.y();
    target_state->target.transform.translation.y = pos.y();
    //target_state->target.transform.translation.y = -pos.x();
    target_state->target.transform.translation.z = pos.z();

    target_state->target.transform.rotation.x = q_rot.x();
    target_state->target.transform.rotation.y = q_rot.y();
    target_state->target.transform.rotation.z = q_rot.z();
    target_state->target.transform.rotation.w = q_rot.w();

    target_state->target.header.frame_id = "optitrack";
    target_state->target.child_frame_id = frame_id;
    target_state->target.header.stamp = ros::Time::now();


    target_state->target_pose_stamped.pose.position.x = pos.x();
    target_state->target_pose_stamped.pose.position.y = pos.y();
    target_state->target_pose_stamped.pose.position.z = pos.z();

    target_state->target_pose_stamped.pose.orientation.x = q_rot.x();
    target_state->target_pose_stamped.pose.orientation.y = q_rot.y();
    target_state->target_pose_stamped.pose.orientation.z = q_rot.z();
    target_state->target_pose_stamped.pose.orientation.w = q_rot.w();

    target_state->target_pose_stamped.header.frame_id = "optitrack";
    target_state->target_pose_stamped.header.stamp = ros::Time::now();

    //NED frame

    target_state->ned_target.transform.translation.x = pos.x();
    //target_state->target.transform.translation.x = pos.y();
    target_state->ned_target.transform.translation.y = -pos.y();
    //target_state->target.transform.translation.y = -pos.x();
    target_state->ned_target.transform.translation.z = -pos.z();

    target_state->ned_target.transform.rotation.x = q_rot.x();
    target_state->ned_target.transform.rotation.y = -q_rot.y();
    target_state->ned_target.transform.rotation.z = -q_rot.z();
    target_state->ned_target.transform.rotation.w = q_rot.w();

    target_state->ned_target.header.frame_id = "optitrack";
    target_state->ned_target.child_frame_id = frame_id;
    target_state->ned_target.header.stamp = ros::Time::now();



    target_state->ned_pose_stamped.pose.position.x = pos.x();
    target_state->ned_pose_stamped.pose.position.y = -pos.y();
    target_state->ned_pose_stamped.pose.position.z = -pos.z();

    target_state->ned_pose_stamped.pose.orientation.x = q_rot.x();
    target_state->ned_pose_stamped.pose.orientation.y = -q_rot.y();
    target_state->ned_pose_stamped.pose.orientation.z = -q_rot.z();
    target_state->ned_pose_stamped.pose.orientation.w = q_rot.w();

    target_state->ned_pose_stamped.header.frame_id = "optitrack";
    target_state->ned_pose_stamped.header.stamp = ros::Time::now();







//        std::cout<<"marker x"<<pos.x()<<std::endl;
//        std::cout<<"marker y"<<pos.y()<<std::endl;
//        std::cout<<"marker z"<<pos.z()<<std::endl;

    fresh_data = true;
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "vrpn_tracked_object_1");
    ros::NodeHandle nh("~");

    target_state = new TargetState;
    //frame_id = nh.getNamespace().substr(1);
    frame_id = nh.getNamespace();

    std::string vrpn_server_ip;
    int vrpn_port;
    std::string tracked_object_name;

    nh.param<std::string>("vrpn_server_ip", vrpn_server_ip, std::string());
    nh.param<int>("vrpn_port", vrpn_port, 3883);

    std::cout<<"vrpn_server_ip:"<<vrpn_server_ip<<std::endl;
    std::cout<<"vrpn_port:"<<vrpn_port<<std::endl;

    Rigid_Body tool(nh, vrpn_server_ip, vrpn_port);

    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        tool.step_vrpn();
        //vrpn_SleepMsecs(10);
        if (fresh_data == true)
        { // only publish when receive data over VRPN.
            tool.publish_target_state(target_state);
            fresh_data = false;
        }
        //ros::spinOnce();
        loop_rate.sleep();
    }
	return 0;
}
