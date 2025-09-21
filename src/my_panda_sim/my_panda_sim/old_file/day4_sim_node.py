#day 4: testing ros node with simple heartbeat
import rclpy                        #ROS2 Python API
from rclpy.node import Node         #Node, every customized node are under this type  
from std_msgs.msg import String     #Message String

class PandaSimNode(Node):
    def __init__(self):
        super().__init__('panda_sim')   # Call Node builder，node name = panda_sim
        self.publisher_ = self.create_publisher(String,             #datatype = string
                                                'heartbeat',    #topic = heartbeat
                                                10)        #if the subscriber cannot collect this node, it will keep at most 10 publishes in the queue
        self.timer = self.create_timer(1.0, self.timer_callback) # per 1.0 sec, call def timer_callback
        self.i = 0

    def timer_callback(self):
        msg = String()                      #make a string
        msg.data = f'Heartbeat #{self.i}'
        self.publisher_.publish(msg)        # publish the message
        self.get_logger().info(f'Publishing: "{msg.data}"')     #And text to terminal for debug
        self.i += 1

def main(args=None):
    rclpy.init(args=args)   #init rclpy
    node = PandaSimNode()   #make node
    rclpy.spin(node)        #start loop to process timer, subscriber. It will keep stucking right here
    node.destroy_node()     #destroy node and release resource
    rclpy.shutdown()

if __name__ == '__main__':
    main()
