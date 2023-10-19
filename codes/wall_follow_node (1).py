import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        self.subscription = self.create_subscription(
                LaserScan,
                lidarscan_topic,
                self.scan_callback,
                10)


        # TODO: create subscribers and publishers 
        self.publisher = self.create_publisher(AckermannDriveStamped , drive_topic, 10)

        # TODO: set PID gains
        self.kp = 5
        self.kd = 0.001
        self.ki = 0.00000001

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.CAR_LENGTH = 0.5

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        #index = int((math.radians(angle)+np.pi)*1080/(2*np.pi))
        index =int(180/45)*angle +180
        print(index)
        range = range_data.ranges[index]
        

        #TODO: implement
        return range

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        a= self.get_range (range_data, 40)
        b= self.get_range (range_data, 0)
        print("a: ", a)
        print("b: ", b)
        alpha =  math.atan2((a*math.cos(math.radians(115))-b),a*math.sin(math.radians(115))) 
        dt = b*math.cos(alpha)
        error = dist-dt
        dt1 = dt + 2*self.CAR_LENGTH*math.sin(alpha)
        
        #TODO:implement
        return dt1

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        self.integral += error * 0.1 #As sleep(0.1) so 0.1sec between scans.
        derivative = (error - self.prev_error) / 0.1
        angle = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        
        
        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 1.5
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 1
        else:
            velocity = 0.5
            
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = float(velocity)
        self.publisher.publish(drive_msg)

    def scan_callback(self, msg):
    
        ranges = [0 for _ in range(3)]
    	
        data = msg.ranges
    	
        ranges[0] = min(data[0:40])
        ranges[1] = min(data[41:80])
        ranges[2] = min(data[81:120])
	
        if ranges.index(min(ranges)) == 0: print("right")
        if ranges.index(min(ranges)) == 1: print("front")
        if ranges.index(min(ranges)) == 2: print("left")
	
        error = self.get_error(msg, 0.5) # TODO: replace with error calculated by get_error()
        print(error)
        
        #velocity = 1.0 # TODO: calculate desired car velocity based on error
        
        self.pid_control(error) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    #minimal_publisher = MinimalPublisher()
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    #rclpy.spin(minimal_publisher)
    #minimal_publisher.destroy_node()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
