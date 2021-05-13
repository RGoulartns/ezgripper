import rclpy
from std_msgs.msg import Int8
from std_srvs.srv import Empty
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy import qos

from ezgripper.libezgripper import Gripper, create_connection
from control_msgs.action import GripperCommand

from functools import partial

class GrippersROS(Node):
    def __init__(self):
        super().__init__('ezgripper_node')

        #ROS2 Parameters
        params = self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyUSB0'),
                ('baudrate', 57600),
                ('default_torque', 50),
                ('names', 'gripper'),
                ('servo_ids', [1])
            ]
        )
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.torque = self.get_parameter('default_torque').value
        self.names = self.get_parameter('names').value
        self.servo_ids = self.get_parameter('servo_ids').value
        
        self.get_logger().info("Port: " + self.port)
        self.get_logger().info("Baud rate: " + str(self.baudrate))
        self.get_logger().info("Default torque: " + str(self.torque))
        self.get_logger().info("Names: " + ' '.join(self.names))
        self.get_logger().info("Servo IDs: " + ' '.join(map(str,self.servo_ids)))
        

        #SAKE EZGripper
        self.grippers = {}
        self.connection = create_connection(dev_name=self.port, baudrate=self.baudrate)
        
        for name, id in zip(self.names, self.servo_ids):
            self.grippers[name] = Gripper(self.connection, name, [id])
            self.calibrate_srv = self.create_service( Empty, '~/'+ name + '/calibrate', partial(self.calibrateSrvCB, name) )
            self.goTo_act = ActionServer( self, GripperCommand, '~/' + name + '/goto', partial(self.gotoActCB, name) )
            
            #just for the tests
            self.torque_sub = self.create_subscription(Int8, '~/'+ name + '/torque', self.setTorqueCB, qos.qos_profile_system_default)
            self.close_srv = self.create_service( Empty, '~/'+ name + '/close', partial(self.closeSrvCB, name) )
            self.open_srv = self.create_service( Empty, '~/'+ name + '/open', partial(self.openSrvCB, name) )

    
    def calibrateSrvCB(self, name, request, response):
        self.calibrate(name)
        return response

    def gotoActCB(self, goal_handler):
        #ToDo
        result = GripperCommand.Result()
        result._stalled = False
        result._reached_goal = True
        return result

    def setTorqueCB(self, msg):
        self.torque = msg.data

    def closeSrvCB(self, name, request, response):
        self.grippers[name].goto_position(0, self.torque)
        return response
    
    def openSrvCB(self, name, request, response):
        self.grippers[name].goto_position(50, self.torque)
        return response

    def calibrate(self, name):
        self.grippers[name].calibrate()
        self.grippers[name].release()
    
    def setPosition(self, pos, torque, name):
        self.grippers[name].goto_position(pos, torque)

    def calibrateAll(self):
        for gripper in self.grippers.values():
            gripper.calibrate()

    def setPositionAll(self, pos, torque):
        for gripper in self.grippers.values():
            gripper.goto_position(pos, torque)



def main(args=None):
    
    rclpy.init(args=args)
    
    ezgripper = GrippersROS()
    ezgripper.calibrateAll()
    ezgripper.setPositionAll(50, ezgripper.torque)

    try:
        rclpy.spin(ezgripper)
    finally:
        ezgripper.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
