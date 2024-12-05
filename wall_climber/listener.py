from rclpy.node import Node
#from std_msgs.msg import String
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import WrenchStamped
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float32MultiArray


class sally_node(Node):
    def __init__(self):
        super().__init__('sally_node')
        self.controls = None
        self.orientation = [0, 0, 0]
        self.acceleration = [0, 0, 0]
 
        self.subscription_joy = self.create_subscription(Joy, 'joy', self.update_controls, 10)
        self.subscription_imu = self.create_subscription(Imu, '/imu/data', self.update_imu, 10)

        self.publisher_1 = self.create_publisher(WrenchStamped, 'contact_force_1', 10)
        self.publisher_2 = self.create_publisher(WrenchStamped, 'contact_force_2', 10)
        self.publisher_3 = self.create_publisher(WrenchStamped, 'contact_force_3', 10)
        self.publisher_4 = self.create_publisher(WrenchStamped, 'contact_force_4', 10)

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
    
    # For publisher
    def publish_contact_forces(self, contact_forces):
        # Split the 1x12 forces of each wheel into four 1x3 segments
        forces_1 = contact_forces[0:3]
        forces_2 = contact_forces[3:6]
        forces_3 = contact_forces[6:9]
        forces_4 = contact_forces[9:12]

        self._publish_force(self.publisher_1, forces_1, "left_drive_1")
        self._publish_force(self.publisher_2, forces_2, "right_drive_2")
        self._publish_force(self.publisher_3, forces_3, "left_drive_3")
        self._publish_force(self.publisher_4, forces_4, "right_drive_4")

    def _publish_force(self, publisher, forces, frame_id):
        """
        Publish contact forces for four contact wheels.
        1 for front left, 2 for front right, 3 for rear left, 4 for rear right
        Splits a 1x12 contact forces array into four 1x3 segments and publishes them
        as WrenchStamped messages on their respective topics.

        Args:
            contact_forces (list or array): A list or array of 12 float values representing contact forces.
        """
        
        msg = WrenchStamped()
        msg.header.frame_id = frame_id  # Frame ID for the corresponding contact point

        # Populate forces and torques
        msg.wrench.force.x = forces[0]
        msg.wrench.force.y = forces[1]
        msg.wrench.force.z = forces[2]
        msg.wrench.torque.x = 0.0  # Assuming no torques for now
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        # Publish
        #publisher.publish(msg)
        #self.get_logger().info(f"Published {frame_id} contact force: {msg.wrench.force}")
        
