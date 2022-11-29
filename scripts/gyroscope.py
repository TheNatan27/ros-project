import asyncio
import websockets
import socket
from base64 import b64decode
import json
import kinpy as kp
from sensor_msgs.msg import JointState
import numpy as np
import rospy
from std_msgs.msg import Float64
import math
import os




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


    def __init__(self):
        self.umbrellaPosition = "up"
        self.oldConfiguration = [0.0,0.0,0.0,0.0]
        self.controller = RRR_arm_controller()

    async def moveUmbrella(self):
        print("Umbrella moving into position")

        controller = RRR_arm_controller()

        rospy.sleep(1.0)
        await controller.to_configuration([1.0,1.0,1.0,0.0])

    async def resetUmbrella(self):
        print("Umbrella moving into default position")

        controller = RRR_arm_controller()

        rospy.sleep(1.0)
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
        print("Connect to " + ipAddress + ":5000")

    def numberLimiter(self, number):
        number = float(number)
        if(number>6.7):
            return 6.7
        if(number<-6.7):
            return -6.7
        else:
            return number

    def xLimiter(self, number):
        number = float(number)
        if(number>2):
            return 2
        if(number<-2):
            return -2
        else:
            return number

    async def getVectors(self, data):
        jsonData = json.loads(data)
        xValue = "{:.4}".format(jsonData['x'])
        yValue = "{:.4}".format(jsonData['y'])
        zValue = "{:.4}".format(jsonData['z'])

        xClear = self.xLimiter(xValue)
        yClear = self.numberLimiter(yValue)
        zClear = self.numberLimiter(zValue)

        print(f'x: {str(xClear)}, y: {str(yClear)}, z: {str(zClear)}')

        print('Vectors created...')
        await self.makeConfiguration(xClear, yClear, zClear)

    async def makeConfiguration(self, x, y, z):
        #rearranging coordinate values
        configuration = [z,x,y,0.0]
        print('Configuration created...')
        await self.compareConfiguration(configuration)

    async def compareConfiguration(self, newConfiguration):
        resultX = newConfiguration[0]-self.oldConfiguration[0]
        resultY = newConfiguration[1]-self.oldConfiguration[1]
        resultZ = newConfiguration[2]-self.oldConfiguration[2]

        resultArray = [resultX, resultY, resultZ]

        print('Comparing configuration differences...')

        didItMoveEnough = self.checkDifference(resultArray)
        didItMove = self.checkForDisplacement(newConfiguration)

        if(didItMove and didItMoveEnough):
            print('Significant difference found!')
            self.oldConfiguration = newConfiguration
            await self.moveArm(newConfiguration)
        else:
            print('Difference is not great enough!')

    def checkDifference(self, resultArray):
        isChangeGreatEnough = False

        for i in resultArray:
            if(abs(i) > 0.2):
                isChangeGreatEnough = True

        print('Difference check: ' + str(isChangeGreatEnough))
        return isChangeGreatEnough

    def checkForDisplacement(self, newConfiguration):
        isDisplaced = False

        for i in newConfiguration:
            if(abs(i) > 0.4):
                isDisplaced = True

        print('Displacement check: ' + str(isDisplaced))
        return isDisplaced

    async def moveArm(self, configuration):
        print('Moving arm...')
        rospy.sleep(0.2)
        await self.controller.to_configuration(configuration)


    async def processData(self, data):
        clear = lambda: os.system('clear')
        clear()
        print('Data received...')
        await self.getVectors(data)
        
        


async def echo(websocket, path):
    async for message in websocket:
        data = await websocket.recv()
        await asl.processData(data)
            

asl = AsyncListener()


asyncio.get_event_loop().run_until_complete(asl.getIp())

asyncio.get_event_loop().run_until_complete(
    websockets.serve(echo, '0.0.0.0', 5000))
asyncio.get_event_loop().run_forever()

 


