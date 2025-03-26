import os
import subprocess
import tempfile
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        
        # Declare parameters
        self.declare_parameter('camera_model', 'Sony Alpha ILCE-6400')
        self.declare_parameter('save_directory', '/media/photos')
        self.declare_parameter('auto_detect', True)
        self.declare_parameter('connection_attempts', 3)
        self.declare_parameter('capture_timeout', 10.0)  # seconds
        
        # Get parameters
        self.camera_model = self.get_parameter('camera_model').value
        self.save_directory = self.get_parameter('save_directory').value
        self.auto_detect = self.get_parameter('auto_detect').value
        self.connection_attempts = self.get_parameter('connection_attempts').value
        self.capture_timeout = self.get_parameter('capture_timeout').value
        
        # Create directories if they don't exist
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Initialize camera variables
        self.camera_port = None
        self.camera_connected = False
        self.last_photo_path = None
        
        # Create subscribers
        self.take_photo_sub = self.create_subscription(
            Int32,
            '/drone/photo/take_photo',
            self.take_photo_callback,
            10
        )
        
        # Create publishers
        self.photo_status_pub = self.create_publisher(
            String,
            '/drone/photo/take_photo/status',
            10
        )
        
        # Initial camera connection
        self.connect_to_camera()
        
        # Create timer for periodic camera checks
        self.connection_timer = self.create_timer(30.0, self.check_camera_connection)
        
        self.get_logger().info('Camera Controller node initialized')
    
    def connect_to_camera(self):
        """Connect to the Sony camera via gphoto2."""
        for attempt in range(self.connection_attempts):
            try:
                self.get_logger().info(f'Attempting to connect to camera (attempt {attempt+1}/{self.connection_attempts})...')
                
                # Auto-detect camera
                if self.auto_detect:
                    result = subprocess.run(['gphoto2', '--auto-detect'], 
                                          capture_output=True, text=True, timeout=5)
                    
                    if self.camera_model in result.stdout:
                        # Camera found
                        self.get_logger().info(f'Camera {self.camera_model} detected')
                        self.camera_connected = True
                        return True
                else:
                    # Check if any camera is connected
                    result = subprocess.run(['gphoto2', '--summary'], 
                                          capture_output=True, text=True, timeout=5)
                    
                    if 'not found' not in result.stderr:
                        self.get_logger().info('Camera connected')
                        self.camera_connected = True
                        return True
                    
            except subprocess.TimeoutExpired:
                self.get_logger().warning('Timeout while connecting to camera')
            except Exception as e:
                self.get_logger().error(f'Error connecting to camera: {str(e)}')
            
            time.sleep(1.0)
        
        self.get_logger().error('Failed to connect to camera after multiple attempts')
        self.camera_connected = False
        return False
    
    def check_camera_connection(self):
        """Periodically check if the camera is still connected."""
        if not self.camera_connected:
            self.connect_to_camera()
        else:
            try:
                # Check if camera is still responding
                result = subprocess.run(['gphoto2', '--summary'], 
                                      capture_output=True, text=True, timeout=5)
                
                if 'not found' in result.stderr or result.returncode != 0:
                    self.get_logger().warning('Camera connection lost, attempting to reconnect...')
                    self.camera_connected = False
                    self.connect_to_camera()
            except Exception as e:
                self.get_logger().warning(f'Error checking camera connection: {str(e)}')
                self.camera_connected = False
    
    def take_photo_callback(self, msg):
        """Callback to take a photo when requested."""
        location_id = msg.data
        self.get_logger().info(f'Received request to take photo at location ID: {location_id}')
        
        success = self.capture_photo(location_id)
        
        # Publish status
        status_msg = String()
        status_msg.data = "success" if success else "failed"
        self.photo_status_pub.publish(status_msg)
    
    def capture_photo(self, location_id):
        """Capture a photo and save it to the specified directory."""
        if not self.camera_connected:
            self.get_logger().error('Cannot take photo: Camera not connected')
            return False
        
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'location_{location_id}_{timestamp}.jpg'
            save_path = os.path.join(self.save_directory, filename)
            
            self.get_logger().info(f'Taking photo: {save_path}')
            
            # Capture the image using gphoto2
            result = subprocess.run(
                ['gphoto2', '--capture-image-and-download', '--filename', save_path],
                capture_output=True,
                text=True,
                timeout=self.capture_timeout
            )
            
            if result.returncode == 0:
                self.get_logger().info(f'Photo captured successfully: {save_path}')
                self.last_photo_path = save_path
                return True
            else:
                self.get_logger().error(f'Failed to capture photo: {result.stderr}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout while capturing photo')
            return False
        except Exception as e:
            self.get_logger().error(f'Error capturing photo: {str(e)}')
            return False
    
    def get_camera_capabilities(self):
        """Get and log camera capabilities."""
        try:
            result = subprocess.run(['gphoto2', '--abilities'], 
                                  capture_output=True, text=True, timeout=5)
            self.get_logger().info(f'Camera capabilities: {result.stdout}')
        except Exception as e:
            self.get_logger().error(f'Error getting camera capabilities: {str(e)}')
    
    def set_camera_config(self, setting, value):
        """Set a camera configuration parameter."""
        try:
            result = subprocess.run(
                ['gphoto2', '--set-config', f'{setting}={value}'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                self.get_logger().info(f'Camera setting {setting} set to {value}')
                return True
            else:
                self.get_logger().error(f'Failed to set camera config: {result.stderr}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error setting camera config: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    camera_controller = CameraController()
    
    try:
        rclpy.spin(camera_controller)
    except KeyboardInterrupt:
        pass
    finally:
        camera_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()