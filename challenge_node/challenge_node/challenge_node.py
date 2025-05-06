import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math



class ChallengeNode(Node):
    def __init__(self):
        super().__init__('challenge_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)

        self.pose = None
        self.scan = None

        self.timer = self.create_timer(0.1, self.execute) 
        
        
        

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def laser_callback(self, msg):
        self.scan = msg.ranges

    def distance_to_goal(self, goal_x, goal_y):
        dx = goal_x - self.pose.position.x
        dy = goal_y - self.pose.position.y
        return math.hypot(dx, dy)

    def angle_to_goal(self, goal_x, goal_y):
        dx = goal_x - self.pose.position.x
        dy = goal_y - self.pose.position.y
        goal_theta = math.atan2(dy, dx)

        q = self.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Normaliza entre -pi e pi
        return math.atan2(math.sin(goal_theta - yaw), math.cos(goal_theta - yaw))
        
        
        

    def front_clear(self, threshold=1.0):
        if self.scan is None:
            return False

        total = len(self.scan)
        center = total // 2
        front_range = self.scan[center - 40:center + 40]  

        filtered = [r for r in front_range if 0.1 < r < float('inf')]
        if not filtered:
            return True  

        return min(filtered) > threshold
        
        
        
        
        

    def side_clear(self, direction='left', threshold=1.1):
        if self.scan is None:
            return False

        total = len(self.scan)
        center = total // 2
        if direction == 'left':
            side_range = self.scan[center + 100:center + 160]  
        elif direction == 'right':
            side_range = self.scan[center - 160:center - 100]  
        else:
            return False

        filtered = [r for r in side_range if 0.1 < r < float('inf')]
        if not filtered:
            return True  

        return min(filtered) > threshold

    def execute(self):
        if self.pose is None or self.scan is None:
            return

        goal_x, goal_y = 4.5, 4.0

        if self.distance_to_goal(goal_x, goal_y) < 0.5:
            self.get_logger().info("Alvo alcançado!")
            self.stop()
            return

        twist = Twist()

        if not self.front_clear():
        
        
            # Obstáculo à frente 
            if self.side_clear('left'):
                twist.linear.x = 0.0
                twist.angular.z = 0.8  
            elif self.side_clear('right'):
                twist.linear.x = 0.0
                twist.angular.z = -0.8  
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.5  
        else:
            # Frente livre → segue para o alvo
            angle = self.angle_to_goal(goal_x, goal_y)
            twist.linear.x = 0.6
            twist.angular.z = max(min(angle * 1.0, 1.0), -1.0) 

        self.publisher_.publish(twist)

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ChallengeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    

if __name__ == '__main__':
    main()

