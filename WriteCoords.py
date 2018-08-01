import rospy
import csv
from nav_msgs.msg import Path           #Change This to the proper type


latestPath = Path()

def callback(msg):
    '''Listens to msg, records latest path

    this approach only works for Path style messages, 

    :param msg: path msg received from ros
    '''
    global latestPath
    latestPath = msg
    # print(msg)


def writePath():
    '''Uses latest message, writes all appropriate data from each pose into a csv

    '''
    global latestPath
    with open('msckf_poses.csv', 'w') as csvFile:
        Wrt = csv.writer(csvFile, delimiter=',')
        Wrt.writerow(['timestamp (nS)', 'pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z' ])
        # print(latestPath)
        initPose = latestPath.poses[0]
        for pose in latestPath.poses:
            # print(pose)
            Wrt.writerow([  str(pose.header.stamp.secs) + '' + str(pose.header.stamp.nsecs),  
                            "%.6f" % (pose.pose.position.x - initPose.pose.position.x),  
                            "%.6f" % (pose.pose.position.y - initPose.pose.position.y),  
                            "%.6f" % (pose.pose.position.z - initPose.pose.position.z), 
                            "%.6f" %  pose.pose.orientation.w,
                            "%.6f" %  pose.pose.orientation.x,
                            "%.6f" %  pose.pose.orientation.y, 
                            "%.6f" %  pose.pose.orientation.z 
                        ])
        print("done")

def init():
    '''creates node

    '''
    rospy.init_node('pathRecorder')
    rospy.Subscriber("/msckf/imu_path", Path, callback)             #change this to proper type, topic
    rospy.on_shutdown(writePath)
    rospy.spin()
init()
