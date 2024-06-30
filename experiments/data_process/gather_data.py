import h5py
import numpy as np
import rospy
import argparse

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from mrs_msgs.msg import PoseWithCovarianceArrayStamped, RangeWithCovarianceArrayStamped, UavState
from nav_msgs.msg import Odometry

file = h5py.File('data.h5', 'w')

file.create_group('object_tracker')
file.create_group('object_tracker/filtered_poses')
file.create_group('uwb')

uwb = None

rtk_leader = None
rtk_follower = None

rtk_leader_velocity = None

def object_tracker_callback(fcu: PoseWithCovarianceArrayStamped, rtk: PoseWithCovarianceArrayStamped, velocity: TwistStamped):
    global file
    global rtk_leader
    global rtk_follower

    if(len(fcu.poses) < 1 or len(rtk.poses) < 1):
        return

    if(rtk_leader is None or rtk_follower is None):
        return

    if(not '/object_tracker/timestamp' in file):
        file['object_tracker'].create_dataset('timestamp', data=[fcu.header.stamp.to_sec()], maxshape=(None,))
    else:
        file['object_tracker/timestamp'].resize((file['object_tracker/timestamp'].shape[0] + 1), axis=0)
        file['object_tracker/timestamp'][-1] = fcu.header.stamp.to_sec()
    
    if(not '/object_tracker/leader' in file):
        file['object_tracker'].create_dataset('leader', data=rtk_leader, maxshape=(None,7))
    else:
        file['object_tracker/leader'].resize((file['object_tracker/leader'].shape[0] + 1), axis=0)
        file['object_tracker/leader'][-1] = rtk_leader

    if(not '/object_tracker/follower' in file):
        file['object_tracker'].create_dataset('follower', data=rtk_follower, maxshape=(None,7))
    else:
        file['object_tracker/follower'].resize((file['object_tracker/follower'].shape[0] + 1), axis=0)
        file['object_tracker/follower'][-1] = rtk_follower

    poses = {"fcu": fcu.poses[0], "rtk": rtk.poses[0]}

    if(not '/object_tracker/velocity_truth' in file):
        file['object_tracker'].create_dataset('velocity_truth', data=rtk_leader_velocity, maxshape=(None,3))
    else:
        file['object_tracker/velocity_truth'].resize((file['object_tracker/velocity_truth'].shape[0] + 1), axis=0)
        file['object_tracker/velocity_truth'][-1] = rtk_leader_velocity

    if(not '/object_tracker/velocity' in file):
        file['object_tracker'].create_dataset('velocity', data=np.array([velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z]).reshape((1,3)), maxshape=(None,3))
    else:
        file['object_tracker/velocity'].resize((file['object_tracker/velocity'].shape[0] + 1), axis=0)
        file['object_tracker/velocity'][-1] = np.array([velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z]).reshape((1,3))
    
    for key, pose in poses.items():
        if( not '/object_tracker/filtered_poses/' + key in file):
            file['object_tracker/filtered_poses'].create_group(key)
            file['object_tracker/filtered_poses/' + key].create_dataset('position', data=np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]).reshape((1, 3)),  maxshape=(None,3))
            file['object_tracker/filtered_poses/' + key].create_dataset('orientation', data=np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]).reshape((1,4)), maxshape=(None,4))
        else:
            file['object_tracker/filtered_poses/' + key + '/position'].resize((file['object_tracker/filtered_poses/' + key + '/position'].shape[0] + 1), axis=0)
            file['object_tracker/filtered_poses/' + key + '/position'][-1] = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]).reshape((1, 3))
            file['object_tracker/filtered_poses/' + key + '/orientation'].resize((file['object_tracker/filtered_poses/' + key + '/orientation'].shape[0] + 1), axis=0)
            file['object_tracker/filtered_poses/' + key + '/orientation'][-1] = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]).reshape((1,4))

        if not '/object_tracker/filtered_poses/' + key + '/covariance' in file:
            file['object_tracker/filtered_poses/' + key].create_dataset('covariance', data=np.array(pose.covariance).reshape((1,36)), maxshape=(None, 36))
        else:
            file['object_tracker/filtered_poses/' + key + '/covariance'].resize((file['object_tracker/filtered_poses/' + key + '/covariance'].shape[0] + 1), axis=0)
            file['object_tracker/filtered_poses/' + key + '/covariance'][-1] = np.array(pose.covariance).reshape((1,36))
    return

def uwb_callback(range: RangeWithCovarianceArrayStamped):
    global file
    global rtk_leader
    global rtk_follower

    if len(range.ranges) < 1:
        return
    
    if(rtk_leader is None or rtk_follower is None):
        return

    if(not '/uwb/timestamp' in file):
        file['uwb'].create_dataset('timestamp', data=[range.header.stamp.to_sec()], maxshape=(None,))
    else:
        file['uwb/timestamp'].resize((file['uwb/timestamp'].shape[0] + 1), axis=0)
        file['uwb/timestamp'][-1] = range.header.stamp.to_sec()

    if(not '/uwb/leader' in file):
        file['uwb'].create_dataset('leader', data=rtk_leader, maxshape=(None,7))
    else:
        file['uwb/leader'].resize((file['uwb/leader'].shape[0] + 1), axis=0)
        file['uwb/leader'][-1] = rtk_leader

    if(not '/uwb/follower' in file):
        file['uwb'].create_dataset('follower', data=rtk_follower, maxshape=(None,7))
    else:
        file['uwb/follower'].resize((file['uwb/follower'].shape[0] + 1), axis=0)
        file['uwb/follower'][-1] = rtk_follower

    if(not '/uwb/range' in file):
        file['uwb'].create_dataset('range', data=np.array(range.ranges[0].range.range).reshape((1,1)), maxshape=(None,1))
        file['uwb'].create_dataset('covariance', data=np.array(range.ranges[0].variance).reshape((1,1)), maxshape=(None,1))
        file['uwb'].create_dataset('power', data=np.array((range.ranges[0].power_a + range.ranges[0].power_b)/2).reshape((1,1)), maxshape=(None,1))
    else:
        file['uwb/range'].resize((file['uwb/range'].shape[0] + 1), axis=0)
        file['uwb/range'][-1] = np.array(range.ranges[0].range.range).reshape((1,1))
        file['uwb/covariance'].resize((file['uwb/covariance'].shape[0] + 1), axis=0)
        file['uwb/covariance'][-1] = np.array(range.ranges[0].variance).reshape((1,1))
        file['uwb/power'].resize((file['uwb/power'].shape[0] + 1), axis=0)
        file['uwb/power'][-1] = np.array((range.ranges[0].power_a + range.ranges[0].power_b)/2).reshape((1,1))

    return

def rtk_leader_callback(odom: UavState):
    global rtk_leader
    global rtk_leader_velocity
    rtk_leader_velocity = np.array([odom.velocity.linear.x, odom.velocity.linear.y, odom.velocity.linear.z]).reshape((1,3))
    rtk_leader = np.array([odom.pose.position.x, 
                           odom.pose.position.y, 
                           odom.pose.position.z, 
                           odom.pose.orientation.x, 
                           odom.pose.orientation.y, 
                           odom.pose.orientation.z, 
                           odom.pose.orientation.w]).reshape((1,7))

def rtk_follower_callback(odom: UavState):
    global rtk_follower
    rtk_follower = np.array([odom.pose.position.x, 
                           odom.pose.position.y, 
                           odom.pose.position.z, 
                           odom.pose.orientation.x, 
                           odom.pose.orientation.y, 
                           odom.pose.orientation.z, 
                           odom.pose.orientation.w]).reshape((1,7))

def main():
    global file
    global rtk_leader
    global rtk_follower
    global uwb

    parser = argparse.ArgumentParser()
    parser.add_argument('file', help='file name to save data to')
    args = parser.parse_args()

    file = h5py.File(args.file, 'w')

    file.create_group('object_tracker')
    file.create_group('object_tracker/filtered_poses')
    file.create_group('uwb')

    rospy.init_node('gather_data')

    object_tracker_fcu_sub = message_filters.Subscriber('/uav42/object_tracker_offline/filtered_poses_fcu', PoseWithCovarianceArrayStamped)
    object_tracker_rtk_sub = message_filters.Subscriber('/uav42/object_tracker_offline/filtered_poses_rtk', PoseWithCovarianceArrayStamped)
    object_tracker_velocity_sub = message_filters.Subscriber('/uav42/object_tracker_offline/filtered_velocity', TwistStamped)

    rtk_leader_sub = rospy.Subscriber('/uav37/estimation_manager/rtk/uav_state', UavState, rtk_leader_callback)
    rtk_follower_sub = rospy.Subscriber('/uav42/estimation_manager/rtk/uav_state', UavState, rtk_follower_callback)

    uwb_sub = rospy.Subscriber('/uav42/uwb_range/range', RangeWithCovarianceArrayStamped, uwb_callback)

    ts = message_filters.TimeSynchronizer([object_tracker_fcu_sub, object_tracker_rtk_sub, object_tracker_velocity_sub], 10)
    ts.registerCallback(object_tracker_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
