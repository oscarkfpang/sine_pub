import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import *
from rcl_interfaces.msg import SetParametersResult

import numpy as np

class SineWavePublisher(Node):

    def __init__(self):
        super().__init__('sine_wave_publisher')
        self.publisher_ = self.create_publisher(SetPosition, 'set_position', 10)
        
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('ampitude', 1.0)
        
        self.min_pos = 1600
        self.max_pos = 2200
        self.n_pos = (self.min_pos + self.max_pos) / 2
        #self.ampitude = (self.max_pos - self.min_pos) / 2
        #self.freq_input = 1 # user's input of frequency - Hz
        #self.period = 1 / self.freq_input
        
        
        self.theta = 0
        #self.d = 10
        

        #self.init_params()
        
        self.freq_input = self.get_parameter('frequency').value
        self.ampitude = self.get_parameter('ampitude').value 
        self.period = 1 / self.freq_input

        self.freq_pub = 20 # publisher frequency - Hz
        timer_period = 1 / self.freq_pub # 0.1  # seconds
        self.d_theta = 2 * np.pi / (self.period * self.freq_pub)
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def parameters_callback(self, params):
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == "frequency":
                self.freq_input = param.value
                self.period = 1 / self.freq_input
                self.d_theta = 2 * np.pi / (self.period * self.freq_pub)
                
            elif param.name == "ampitude":
                if param.value >=0.0 and param.value <=1.0:
                    self.ampitude = param.value
        return SetParametersResult(successful=True)

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', rclpy.Parameter.Type.DOUBLE),
                ('ampitude', rclpy.Parameter.Type.DOUBLE),
            ],
        )


        param = self.get_parameter('frequency')
        self.get_logger().info(f'{param.name}={param.value}')
        self.freq_input = param.value
        self.period = 1 / self.freq_input

        param = self.get_parameter('ampitude')
        self.get_logger().info(f'{param.name}={param.value}')
        self.ampitude = param.value * (self.max_pos - self.min_pos) / 2

        self.add_on_set_parameters_callback(self.on_params_changed)


    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'frequency':
                self.freq_input = param.value
            elif param.name == 'ampitude':
                self.ampitude = param.value
            else:
                continue

        #return SetParametersResult(succesful=True, reason='Parameter set')
    

    

    def timer_callback(self):
        msg = SetPosition()
        msg.id = 0
            
        #msg.position = self.i
        msg.position = round(np.sin(self.theta) * self.ampitude * (self.max_pos - self.min_pos) / 2 + self.n_pos)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing pos: "%s"  theta: "%s"' % (msg.position, self.theta))
        
        self.theta += self.d_theta
        
        # clippig of theta to [0, 2Pi]
        if self.theta >= 2*np.pi:
            self.theta = 0

def main(args=None):
    rclpy.init(args=args)

    node = SineWavePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
