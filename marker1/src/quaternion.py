from tf.transformations import quaternion_from_euler
from tf.transformations import *
import math
from geometry_msgs.msg import Quaternion

if __name__ == '__main__':

	# RPY to convert: 90deg, 0, -90deg
	q = quaternion_from_euler(1.5707, 1.5707, 00)

	print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])

	# q_orig = quaternion_from_euler(0, 0, 0)
	# q_rot = quaternion_from_euler(math.pi, 0, 0)
	# q_new = quaternion_multiply(q_rot, q_orig)
	# print q_new