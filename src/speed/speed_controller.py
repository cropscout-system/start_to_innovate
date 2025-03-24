import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Float32  # Changed from Int32 to Float32

class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')

        # Create a client for the CommandLong service
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # Subscriber to listen to the /drone/speed topic
        self.speed_sub = self.create_subscription(
            Float32,  # Changed to Float32
            '/drone/speed',  # Topic name
            self.speed_callback,  # Callback function
            1  # Queue size
        )

        while not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('CommandLong service not available, waiting...')

        # Log that the node has started
        self.get_logger().info('Speed Controller node started. Listening to /drone/speed...')

        


    def speed_callback(self, msg):
        """
        Callback function for the /drone/speed topic.
        """
        speed = float(msg.data)  # Extract the speed value from the message
        self.get_logger().info(f'Received new speed: {speed} m/s')
        self.set_speed(speed)  # Call the set_speed function


    def set_speed(self, speed):
        """
        Sends the MAV_CMD_DO_CHANGE_SPEED command using the CommandLong service.
        """
        # Wait for the CommandLong service to be available
        

        # Create a CommandLong request
        request = CommandLong.Request()
        request.command = 178  # MAV_CMD_DO_CHANGE_SPEED
        request.confirmation = 0
        request.param1 = 1.0  # Speed type (0 = airspeed, 1 = ground speed)
        request.param2 = speed  # Target speed (m/s)
        request.param3 = -1.0  # Throttle (-1 = no change)
        request.param4 = 0.0  # Relative (0 = absolute, 1 = relative)
        #  FLOAT VALUES IS NECESSARRY !!!

        # Call the CommandLong service asynchronously
        future = self.command_client.call_async(request)
        future.add_done_callback(lambda future: self.command_callback(future, speed))


    def command_callback(self, future, speed):
        """
        Callback function for the CommandLong service call.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Speed set to {speed} m/s successfully!')
            else:
                self.get_logger().error('Failed to set speed. Command was not successful.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    speed_controller = SpeedController()
    rclpy.spin(speed_controller)
    speed_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()