#USING CONVENTION FOR QUATERNION: (w,x,y,z)
#START: 1526349215296000004
#END:   1526349801332000017

from std_msgs.msg import String
import roslaunch
import rospy
import csv
from scipy import optimize
from scipy.optimize import basinhopping
import random
import math
import os
from nav_msgs.msg import Path
import numpy as np

latestMsg = Path()
endStamp = 1526349801332000017
minJump = 1
maxJumps = 15
ConfIters = 3
bestCost = 0

def try_quaternions():
    quatRA = []
    displacement = []
    together = []
    global bestCost
    result = readCost()
    x,bestCost = [float(i) for i in result[:-1]], float(result[-1])
    minimizer_kwargs = {"method":"BFGS"}
    mybounds = MyBounds()

    ret = basinhopping(costFunction, x, minimizer_kwargs=minimizer_kwargs, niter=200, accept_test=mybounds)
    print(ret)

def writeRun(params, cost):
    writePath = '/home/alanhernandez/msckf_node/src/msckf_mono/launch/bestRuns2.csv'
    fh = open(writePath, "a")
    writer = csv.writer(fh)
    params = (list(params))
    params.append(cost)
    writer.writerow(params)
    fh.close()

    return params
def readCost():
    readPath = '/home/alanhernandez/msckf_node/src/msckf_mono/launch/bestRuns2.csv'
    fh = open(readPath, 'r')
    reader = csv.reader(fh)
    params = (list(reader)[-1])
    fh.close()
    return params
    
def writeParams(params):
    writePath = '/home/alanhernandez/msckf_node/src/msckf_mono/launch/params.ini'

    print(params)
    
    fh = open(writePath, "w")
    fh.write("translation_threshold(float)\n")
    fh.write(str(params[0]) + "\n")
    fh.write("keyframe_transl_dist(float)\n")
    fh.write(str(params[1]) + "\n")
    fh.write("keyframe_rot_dist(float)\n")
    fh.write(str(params[2]) + "\n")
    fh.close()

    return params

def costFunction(params):
    params = np.absolute(params)
    writeParams(params)
    minCost = 100000000000
    for i in range(ConfIters):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/alanhernandez/msckf_node/src/msckf_mono/launch/asl_msckf.launch"])
        launch.start()
        displacement = listener()
        launch.shutdown()
        global latestMsg
        latestMsg = Path()
        minCost = minCost if minCost < displacement else displacement
        print(i)

    global bestCost
    if(minCost < bestCost):
        writeRun(params, minCost)
        global bestCost
        bestCost = minCost

    #Calculate the cost
    return minCost

def callback(data):
    '''callback for rospy subscriber, just records most recent message
    '''
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    global latestMsg
    latestMsg = data

def listener():
    '''Creates a rospy node to record the data from msckf, waits until the messages pass a certain timestamp,
    then returns the xyz displacement of the imu path
    :return tuple: x, y, z displacement of imu_path after begining 

    '''
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber("/msckf/imu_path", Path, callback)
    start = rospy.get_time()
    
        #sum up timestamp to nanoseconds
    global latestMsg

    displacement = 0
    maxThreshold = 200
    minThreshold = -1
    cost = 0
    speed = 0
    jumps = 0
    withinBounds = True
    currentTime = 0.0
    reason = ''
    while withinBounds and jumps < maxJumps: #wait until timestamp is reached
        if (rospy.get_time() - start > 10) and (sub.get_num_connections() < 1):
            break
        currentTime = rospy.get_time() - start
        if(currentTime > 50.0):
            global maxJumps
            maxJumps = 25
            
        if(len(latestMsg.poses) > 1):
            displacementX = latestMsg.poses[-1].pose.position.x - latestMsg.poses[0].pose.position.x
            displacementY = latestMsg.poses[-1].pose.position.y - latestMsg.poses[0].pose.position.y
            if(math.fabs(displacementX) < 100 and math.fabs(displacementY) < 300):
                withinBounds = True
            else:
                withinBounds = False
                reason = 'too much x disp ' + str(displacementX) if math.fabs(displacementX) > 100 else 'too much y' + str(displacementY)
            displacement = magnitude([latestMsg.poses[-1].pose.position.x - latestMsg.poses[0].pose.position.x,
            latestMsg.poses[-1].pose.position.y - latestMsg.poses[0].pose.position.y,
            latestMsg.poses[-1].pose.position.z - latestMsg.poses[0].pose.position.z])
            minThreshold = 10
        if(len(latestMsg.poses) > 10):
            jumpDist = magnitude([latestMsg.poses[-1].pose.position.x  - latestMsg.poses[-2].pose.position.x,
            latestMsg.poses[-1].pose.position.y - latestMsg.poses[-2].pose.position.y,
            latestMsg.poses[-1].pose.position.z - latestMsg.poses[-2].pose.position.z])

            if jumpDist > minJump:
                jumps += 1
                if jumps >= 10:
                    reason = 'too many jumps ' + str(jumps)
            
        #print(msgTime)

        #return final displacements Pos_f - Pos_i in x,y,z tuple
    print('died due to ' + reason)
    print('died at' + str(rospy.get_time() - start))
    sub.unregister()
    return(((1 / (rospy.get_time() - start)) * 1000)** 2)

def magnitude(quat):
    summation = 0
    mag = 0
    
    for e in quat:
        summation = summation  + (e * e)
        
    mag = math.sqrt(summation)
    return mag

class MyBounds(object):
    def __init__(self, xmax=(1.0,1.0,1.0),xmin=(0.0,0.0,0.0)): 
        self.xmax = np.array(xmax)

        self.xmin = np.array(xmin)

    def __call__(self, **kwargs):
        x = kwargs["x_new"]
        tmax = bool(np.all(x <= self.xmax))
        tmin = bool(np.all(x >= self.xmin))

        return tmax and tmin

if __name__ == "__main__":
    try:
        try_quaternions()
    except(Exception):
        import traceback
	traceback.print_exc()
    else:
        print("Finished Optimization \n")
