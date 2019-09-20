#!/usr/bin/env python
import kdl_parser_py.urdf  
import PyKDL as kdl
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
#setup KDL
filename ="/home/murakami/catkin_ws/src/nexis_rviz/urdf/arm_test.urdf"
(ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
print ok,tree
chain = tree.getChain("world", "body3_link")  
t = kdl.Tree(tree)  
fksolverpos = kdl.ChainFkSolverPos_recursive(chain)  
iksolvervel = kdl.ChainIkSolverVel_pinv(chain)  
iksolverpos = kdl.ChainIkSolverPos_NR(chain,fksolverpos,iksolvervel)

    
rospy.init_node('jointstate_tip', anonymous=True)
pub = rospy.Publisher('tip_status', JointState, queue_size=10)

position = [0] * 5
position_old = [0] * 5
q_zero = None
q_zero=kdl.JntArray(chain.getNrOfJoints())  
q_solved = None
def first_test(msg):
	global q_zero
	global q_solved
	#set initila joint argument
	#q_init=kdl.JntArray(chain.getNrOfJoints())
	for i in range(5):
		q_init[i] = msg.data[i]

	#set initila joint argument

	q_solved=kdl.JntArray(chain.getNrOfJoints())
	F1=kdl.Frame.Identity()
	#joint angle to cartesian coordinate
	fksolverpos.JntToCart(q_init,F1)
	print "F1.p",F1.p
	print F1.M.GetQuaternion()
	msg.data[0] = F1.p[0]
	msg.data[1] = F1.p[1]
	msg.data[2] = F1.p[2]
	data = [0,0,0,0]
	data = F1.M.GetQuaternion()
	print msg.data[3]
	print msg.data[4]
	print msg.data[5]
	print msg.data[6]
	pub.publish(msg)	
	#cartesian coordinate to joint anlge
	#q_zero=kdl.JntArray(chain.getNrOfJoints())
	#iksolverpos.CartToJnt(q_zero,F1,q_solved)
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
	rospy.Subscriber("dynamixel_jointstates", Float32MultiArray, first_test)
	rospy.spin()

        
if __name__ == '__main__':
    listener()