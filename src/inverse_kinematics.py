#!/usr/bin/env python
import kdl_parser_py.urdf  
import PyKDL as kdl
import glob
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
#setup KDL
filename ="/home/murakami/catkin_ws/src/nexis_rviz/urdf/arm_test.urdf"
#filename = rospy.get_param("cont")
(ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
print ok,tree
chain = tree.getChain("world", "body3_link")  
t = kdl.Tree(tree)  
fksolverpos = kdl.ChainFkSolverPos_recursive(chain)  
iksolvervel = kdl.ChainIkSolverVel_pinv(chain)  
iksolverpos = kdl.ChainIkSolverPos_NR(chain,fksolverpos,iksolvervel)
flg = [True] * 5 
for i in range(5):
    flg.append(True)
def callback(data):
    caliculate(data.data[0],data.data[1],data.data[2])
    
rospy.init_node('inverse_kinematics', anonymous=True)

rospy.Subscriber("chatter", Float32MultiArray, callback)
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

position = [2] * 5
position_old = [2] * 5
q_zero = None
q_zero=kdl.JntArray(chain.getNrOfJoints())  
q_solved = None
def caliculate(data0, data1, data2):
    global q_zero
    global q_solved
    print q_zero
    F2 = kdl.Frame(kdl.Rotation.Quaternion(0,-0.29552,0,0.955337),kdl.Vector(data0,data1,data2)) 
    global position
    global position_old
   # print position[0]
    #print position_old[0]
    for i in range(5):
        if flg[i] == True:
            position_old[i] = position[i]
    q_solved=kdl.JntArray(chain.getNrOfJoints())  
    iksolverpos.CartToJnt(q_zero,F2,q_solved)  
    print "joint angles"  
    print q_solved  
    q_zero = q_solved
    
    position = q_solved
    #r = rospy.Rate(10) # 10hz
    js0 = JointState()
    js0.header.stamp = rospy.Time.now()
    js0.name = ["body0_joint_yaw", "body1_joint", "body2_joint", "body3_joint_yaw", "body3_joint"]
    js0.position = [0] * 5
    for i in range(5):
       # if (position_old[i] - position[i]) < abs(0.1744):
        if 1:
            js0.position[i] = (position[i])%(2*3.14159265)
            flg[i] = True
        else:
            flg[i] = False
    #js0.position = [position[0], position[1], position[2], position[3], position[4]]
    print js0.position
    pub.publish(js0)
    #r.sleep()
def first_test():
    #set initila joint argument
    q_init=kdl.JntArray(chain.getNrOfJoints())
    q_init[0] = 0.0
    q_init[1] = 5.707
    q_init[2] = 2.3096
    q_init[3] = 0
    q_init[4] = 3.9494

    q_solved=kdl.JntArray(chain.getNrOfJoints())
    F1=kdl.Frame.Identity()
    #joint angle to cartesian coordinate
    fksolverpos.JntToCart(q_init,F1)
   # print "F1.p",F1.p
    #print F1.M.GetQuaternion()
    print F1.p[0]
    print F1.p[1]
    print F1.p[2]
    data = [0,0,0,0]
    data = F1.M.GetQuaternion()
    print data[0]
    print data[1]
    print data[2]
    print data[3]
    #F1.M.GetQuaternion(data[0],data[1],data[2],data[3] )
    
    #cartesian coordinate to joint anlge
    q_zero=kdl.JntArray(chain.getNrOfJoints())
    iksolverpos.CartToJnt(q_zero,F1,q_solved)
    #print q_solved

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    
    # spin() simply keeps python from exiting until this node is stopped
    #caliculate(0.1224, 0, 0.35447)
    #first_test()
    rospy.spin()

        
if __name__ == '__main__':
    listener()