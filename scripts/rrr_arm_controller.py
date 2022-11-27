import asyncio
import websockets
import socket
from base64 import b64decode
import wave
import json
import kinpy as kp
from sensor_msgs.msg import JointState
import numpy as np
import math
import rospy
from std_msgs.msg import Float64

class RRR_arm_controller:
    def __init__(self):
        rospy.init_node('rrr_arm_controller', anonymous=True)
        rospy.Subscriber("/rrr_arm/joint_states", JointState, self.cb_q_curr)

        self.joint_1_pub = rospy.Publisher(
                '/rrr_arm/joint1_position_controller/command',
                Float64, queue_size=10)

        self.joint_2_pub = rospy.Publisher(
                '/rrr_arm/joint2_position_controller/command',
                Float64, queue_size=10)

        self.joint_3_pub = rospy.Publisher(
                '/rrr_arm/joint3_position_controller/command',
                Float64, queue_size=10)

        self.joint_4_pub = rospy.Publisher(
                '/rrr_arm/joint4_position_controller/command',
                Float64, queue_size=10)

        self.chain = kp.build_serial_chain_from_urdf(open("/home/ros-user/catkin_ws/src/rrr-arm/urdf/rrr_arm.xacro.urdf")
                .read(), "gripper_frame_cp")

    def cb_q_curr(self,msg):
        self.q_curr = np.concatenate((np.array(msg.position[2:6]),
                np.array(msg.position[6:2])))

    async def to_configuration(self, q):
        self.joint_1_pub.publish(Float64(q[0]))
        self.joint_2_pub.publish(Float64(q[1]))
        self.joint_3_pub.publish(Float64(q[2]))
        self.joint_4_pub.publish(Float64(q[3]))

class AsyncListener:

    async def moveAsync(self):
        print("Hello asy")

        controller = RRR_arm_controller()

        rospy.sleep(1.0)
        await controller.to_configuration([1.0,1.0,1.0,0.5])
        reset = input('Press enter to reset')
        await controller.to_configuration([0.0,0.0,0.0,0.0])


    async def getIp(self):
        hostname = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            hostname.connect(('10.255.255.255', 1))
            ipAddress = hostname.getsockname()[0]
        except Exception:
            ipAddress = '127.0.0.1'
        finally:
            hostname.close()
        return ipAddress

    

if __name__ ==  '__main__':

    asl = AsyncListener()
    print('hello main')

    loop = asyncio.get_event_loop()
    print(loop.run_until_complete(asl.getIp()))
    loop.run_until_complete(asl.moveAsync())


