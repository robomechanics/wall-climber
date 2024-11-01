from rclpy.node import Node
#from std_msgs.msg import String
from sensor_msgs.msg import Joy

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.controls = None
        self.subscription = self.create_subscription(Joy, 'joy', self.callback, 10)
        
    def callback(self, data):
        self.controls = (data.axes, data.buttons)
        #print(self.controls)

    def get_data(self):
        return self.controls
