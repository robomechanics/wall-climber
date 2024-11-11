from rclpy.node import Node
#from std_msgs.msg import String
from sensor_msgs.msg import Joy, Imu
from scipy.spatial.transform import Rotation

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.controls = None
        self.orientation = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.subscription_joy = self.create_subscription(Joy, 'joy', self.update_controls, 10)
        self.subscription_imu = self.create_subscription(Imu, '/imu/data', self.update_imu, 10)

    def update_controls(self, data):
        self.controls = (data.axes, data.buttons)
        #print(self.controls)

    def get_data(self):
        return self.controls
    
    def update_imu(self, data): 
        #q = np.ndarray(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        q = Rotation.from_quat([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        e = q.as_euler('xyz', True)
        self.orientation = [float(e[0]) % 360, float(e[1]) % 360, float(e[2]) % 360]
        
        """
        Adding this line to obtain acceleration
        """
        
        self.acceleration = [
            -data.linear_acceleration.x,
            -data.linear_acceleration.y,
            -data.linear_acceleration.z
        ]

    def get_orientation(self):
        return self.orientation

    def get_acceleration(self):
        return self.acceleration
