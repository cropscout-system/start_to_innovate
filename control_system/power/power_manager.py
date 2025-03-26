#!/usr/bin/env python3

import os
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class PowerManager(Node):
    def __init__(self):
        super().__init__('power_manager')
        
        # Declare parameters with default values
        self.declare_parameter('battery_warning_threshold', 11.5)  # Voltage for warning
        self.declare_parameter('battery_critical_threshold', 10.8)  # Voltage for critical action
        self.declare_parameter('check_interval', 10.0)  # Battery check interval in seconds
        self.declare_parameter('rpi_battery_monitor', True)  # Whether to use Pi-specific monitoring
        self.declare_parameter('publish_voltage', True)  # Whether to publish voltage as a topic
        self.declare_parameter('simulate_battery', False)  # For testing without hardware
        self.declare_parameter('simulate_voltage', 12.6)  # Starting voltage for simulation
        
        # Get parameters
        self.battery_warning_threshold = self.get_parameter('battery_warning_threshold').value
        self.battery_critical_threshold = self.get_parameter('battery_critical_threshold').value
        self.check_interval = self.get_parameter('check_interval').value
        self.rpi_battery_monitor = self.get_parameter('rpi_battery_monitor').value
        self.publish_voltage = self.get_parameter('publish_voltage').value
        self.simulate_battery = self.get_parameter('simulate_battery').value
        self.simulate_voltage = self.get_parameter('simulate_voltage').value
        
        # Internal state
        self.warning_issued = False
        self.return_home_issued = False
        self.current_voltage = self.simulate_voltage if self.simulate_battery else None
        
        # Publishers
        self.voltage_pub = self.create_publisher(Float32, '/drone/power/voltage', 10)
        self.return_home_pub = self.create_publisher(Bool, '/drone/return_home', 10)
        self.power_status_pub = self.create_publisher(String, '/drone/power/status', 10)
        
        # Create timers
        self.battery_check_timer = self.create_timer(self.check_interval, self.check_battery)
        
        if self.simulate_battery:
            # Simulate battery drain when in simulation mode
            self.simulate_timer = self.create_timer(5.0, self.simulate_battery_drain)
            self.get_logger().info(f'Starting with simulated battery voltage: {self.simulate_voltage}V')
        
        self.get_logger().info('Power Manager node initialized')
        self.get_logger().info(f'Battery warning threshold: {self.battery_warning_threshold}V')
        self.get_logger().info(f'Battery critical threshold: {self.battery_critical_threshold}V')
    
    def check_battery(self):
        """Check battery voltage and issue warnings/commands as needed."""
        voltage = self.get_battery_voltage()
        
        if voltage is None:
            self.get_logger().warning('Could not retrieve battery voltage')
            return
        
        self.current_voltage = voltage
        
        # Log current voltage
        self.get_logger().info(f'Current battery voltage: {voltage:.2f}V')
        
        # Publish voltage if enabled
        if self.publish_voltage:
            msg = Float32()
            msg.data = float(voltage)
            self.voltage_pub.publish(msg)
        
        # Check thresholds and issue warnings/commands
        if voltage <= self.battery_critical_threshold and not self.return_home_issued:
            # CRITICAL - Issue return-to-home command
            self.get_logger().error(f'CRITICAL BATTERY LEVEL: {voltage:.2f}V')
            self.trigger_return_home()
            self.return_home_issued = True
            
            status_msg = String()
            status_msg.data = "CRITICAL"
            self.power_status_pub.publish(status_msg)
            
        elif voltage <= self.battery_warning_threshold and not self.warning_issued:
            # WARNING - Issue warning only
            self.get_logger().warning(f'LOW BATTERY WARNING: {voltage:.2f}V')
            self.warning_issued = True
            
            status_msg = String()
            status_msg.data = "WARNING"
            self.power_status_pub.publish(status_msg)
            
        elif voltage > self.battery_warning_threshold and (self.warning_issued or self.return_home_issued):
            # Reset warnings if voltage recovers (useful for testing)
            self.get_logger().info('Battery voltage recovered')
            self.warning_issued = False
            self.return_home_issued = False
            
            status_msg = String()
            status_msg.data = "NORMAL"
            self.power_status_pub.publish(status_msg)
    
    def get_battery_voltage(self):
        """Get the current battery voltage."""
        if self.simulate_battery:
            return self.simulate_voltage
        
        if self.rpi_battery_monitor:
            return self.get_raspberry_pi_voltage()
        else:
            # Placeholder for other voltage monitoring methods
            self.get_logger().warning('No voltage monitoring method available')
            return None
    
    def get_raspberry_pi_voltage(self):
        """Get voltage information from Raspberry Pi using vcgencmd."""
        try:
            # Try to get voltage using vcgencmd
            result = subprocess.run(
                ['vcgencmd', 'measure_volts', 'core'], 
                capture_output=True, 
                text=True, 
                timeout=2.0
            )
            
            if result.returncode == 0:
                # Extract voltage from output like "volt=1.20V"
                match = re.search(r'volt=(\d+\.\d+)V', result.stdout)
                if match:
                    core_voltage = float(match.group(1))
                    # Raspberry Pi core voltage is typically around 1.2V
                    # For a battery monitor, we'd need to scale this to the actual battery voltage
                    # This is a placeholder - in real use you would read from a connected ADC
                    
                    # For demonstration, let's assume a scaling factor to simulate a battery
                    # This would be replaced by actual ADC readings in a real application
                    return core_voltage * 10.0
            
            # If vcgencmd fails, try reading from sysfs if available (for Pi with battery management HATs)
            if os.path.exists('/sys/class/power_supply/battery/voltage_now'):
                with open('/sys/class/power_supply/battery/voltage_now', 'r') as f:
                    # voltage_now is typically in microvolts
                    voltage_uv = int(f.read().strip())
                    return voltage_uv / 1000000.0  # Convert to volts
            
            self.get_logger().warning('Failed to get voltage from Raspberry Pi')
            return None
            
        except Exception as e:
            self.get_logger().error(f'Error getting Raspberry Pi voltage: {str(e)}')
            return None
    
    def simulate_battery_drain(self):
        """Simulate battery drain over time (for testing without hardware)."""
        # Decrease voltage slightly each time
        drain_rate = 0.01  # Volts per interval
        self.simulate_voltage -= drain_rate
        
        # Don't let simulated voltage go below a minimum
        if self.simulate_voltage < 9.0:
            self.simulate_voltage = 9.0
    
    def trigger_return_home(self):
        """Send command to make the drone return home."""
        self.get_logger().info('Triggering return-to-home due to low battery')
        
        # Publish the return home command
        msg = Bool()
        msg.data = True
        self.return_home_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    power_manager = PowerManager()
    
    try:
        rclpy.spin(power_manager)
    except KeyboardInterrupt:
        pass
    finally:
        power_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
