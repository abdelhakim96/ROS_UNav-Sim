# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from bluerov2_nmpc.msg import PwmCmd
#from bluerov2_nmpc.msg import PwmCmd
import numpy as np
import array as arr

wrench_desired = Wrench()
pwm_cmd_msg= PwmCmd()
def wrench_cb(data):
    
    wrench_desired.force.x = data.data[0]
    wrench_desired.force.y = data.data[1]
    wrench_desired.force.z = data.data[2]
    wrench_desired.torque.z = -data.data[3]

    print(wrench_desired)
    


def allocator():
    pub = rospy.Publisher('/airsim_node/RovSimple/pwm_cmd', PwmCmd, queue_size=10)
    rospy.Subscriber("/bluerov2/wrench", Float64MultiArray, wrench_cb)

    rospy.init_node('bluerov2_thrust_allocator_node', anonymous=True)
    rate = rospy.Rate(200) # 10hz

    #A = np.array([[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, -0.5, -0.5], [0.0, 0.0, 0.0, 0.0, -0.5, 0.5,0.5,-0.5],[1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0, 0.025, -0.025, 0.025, -0.025]]) # Array of floats
    
    A = np.array([[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, -0.5, -0.5], [0.0, 0.0, 0.0, 0.0, -0.5, 0.5,0.5,-0.5],[1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0, 0.1, -0.1, 0.1, -0.1]]) # Array of floats
    A_inv = np.linalg.pinv(A)
        
    v=[ [wrench_desired.force.x], [wrench_desired.force.y], [wrench_desired.force.z], [wrench_desired.torque.z]]
    pwm_out = A_inv @ v
    pwm_out = pwm_out.ravel()
    #pwm_out = pwm_out.transpose()
    pwm_out = list (pwm_out)
    pwm_cmd_msg.pwmvals= pwm_out
    #print(pwm_out)
    

    #print(pwm_out)
    while not rospy.is_shutdown():
          rate.sleep()
          v=[ [wrench_desired.force.x], [wrench_desired.force.y], [-wrench_desired.force.z], [wrench_desired.torque.z]]
          pwm_out = A_inv @ v

          #pwm_out = pwm_out.ravel()
          #pwm_out = list (pwm_out)
    
          pwm_cmd_msg.pwmvals= pwm_out
          pub.publish(pwm_cmd_msg)

if __name__ == '__main__':
    try:
        allocator()
    except rospy.ROSInterruptException:
        pass
