#!/usr/bin/env python3

"""
Unit tests for drone control tools
"""

import unittest
from unittest.mock import Mock, MagicMock, patch
import time
from drone_tools import create_tools, VEHICLE_CMD_COMPONENT_ARM_DISARM, VEHICLE_CMD_NAV_LAND

class TestDroneTools(unittest.TestCase):
    """Test cases for drone control tools"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create mock ROS node
        self.mock_node = Mock()
        self.mock_node.vehicle_command_pub = Mock()
        self.mock_node.trajectory_pub = Mock()
        self.mock_node.current_status = {
            'armed': False,
            'mode': 'MANUAL',
            'battery_percentage': 100,
            'position': [0.0, 0.0, 0.0],
            'is_landed': True
        }
        
        # Test config
        self.config = {
            'safety': {
                'max_altitude': 50.0,
                'min_altitude': 0.5,
                'max_distance': 100.0
            },
            'publishing': {
                'trajectory_repeat': 2,  # Reduced for faster tests
                'position_repeat': 2,
                'publish_delay': 0.01   # Faster for tests
            },
            'ros2': {
                'system_id': 1,
                'component_id': 1
            }
        }
        
        # Create tools
        self.tools = create_tools(self.mock_node, self.config)
        self.tools_dict = {tool.__name__: tool for tool in self.tools}
    
    def test_arm_drone(self):
        """Test arm_drone function"""
        result = self.tools_dict['arm_drone']()
        
        # Check that command was published
        self.mock_node.vehicle_command_pub.publish.assert_called_once()
        
        # Check the message
        call_args = self.mock_node.vehicle_command_pub.publish.call_args[0][0]
        self.assertEqual(call_args.param1, 1.0)  # Arm command
        self.assertEqual(call_args.command, VEHICLE_CMD_COMPONENT_ARM_DISARM)
        
        self.assertIn("Arm command sent", result)
    
    def test_disarm_drone(self):
        """Test disarm_drone function"""
        result = self.tools_dict['disarm_drone']()
        
        # Check that command was published
        self.mock_node.vehicle_command_pub.publish.assert_called_once()
        
        # Check the message
        call_args = self.mock_node.vehicle_command_pub.publish.call_args[0][0]
        self.assertEqual(call_args.param1, 0.0)  # Disarm command
        
        self.assertIn("Disarm command sent", result)
    
    def test_takeoff_valid_altitude(self):
        """Test takeoff with valid altitude"""
        result = self.tools_dict['takeoff'](5.0)
        
        # Should publish trajectory multiple times
        self.assertEqual(
            self.mock_node.trajectory_pub.publish.call_count,
            self.config['publishing']['trajectory_repeat']
        )
        
        # Check the message
        call_args = self.mock_node.trajectory_pub.publish.call_args[0][0]
        self.assertEqual(call_args.position, [0.0, 0.0, -5.0])  # NED frame
        
        self.assertIn("Takeoff command sent for altitude 5.0 meters", result)
    
    def test_takeoff_invalid_altitude(self):
        """Test takeoff with invalid altitude"""
        # Too low
        result = self.tools_dict['takeoff'](0.1)
        self.assertIn("Invalid altitude", result)
        self.mock_node.trajectory_pub.publish.assert_not_called()
        
        # Too high
        self.mock_node.trajectory_pub.publish.reset_mock()
        result = self.tools_dict['takeoff'](100.0)
        self.assertIn("Invalid altitude", result)
        self.mock_node.trajectory_pub.publish.assert_not_called()
    
    def test_goto_position_valid(self):
        """Test goto_position with valid coordinates"""
        result = self.tools_dict['goto_position'](10.0, 20.0, 5.0)
        
        # Should publish position multiple times
        self.assertEqual(
            self.mock_node.trajectory_pub.publish.call_count,
            self.config['publishing']['position_repeat']
        )
        
        # Check the message
        call_args = self.mock_node.trajectory_pub.publish.call_args[0][0]
        self.assertEqual(call_args.position, [10.0, 20.0, -5.0])  # NED frame
        
        self.assertIn("Moving to position", result)
    
    def test_goto_position_invalid(self):
        """Test goto_position with invalid coordinates"""
        # Too far
        result = self.tools_dict['goto_position'](200.0, 0.0, 5.0)
        self.assertIn("too far from home", result)
        self.mock_node.trajectory_pub.publish.assert_not_called()
        
        # Invalid altitude
        self.mock_node.trajectory_pub.publish.reset_mock()
        result = self.tools_dict['goto_position'](10.0, 10.0, 0.1)
        self.assertIn("Altitude must be between", result)
        self.mock_node.trajectory_pub.publish.assert_not_called()
    
    def test_land(self):
        """Test land function"""
        result = self.tools_dict['land']()
        
        # Check that command was published
        self.mock_node.vehicle_command_pub.publish.assert_called_once()
        
        # Check the message
        call_args = self.mock_node.vehicle_command_pub.publish.call_args[0][0]
        self.assertEqual(call_args.command, VEHICLE_CMD_NAV_LAND)
        
        self.assertIn("Land command sent", result)
    
    def test_get_status(self):
        """Test get_status function"""
        result = self.tools_dict['get_status']()
        
        # Check that status includes all expected information
        self.assertIn("Armed: False", result)
        self.assertIn("Mode: MANUAL", result)
        self.assertIn("Battery: 100%", result)
        self.assertIn("Position:", result)
        self.assertIn("Landed: True", result)
    
    def test_hold_position(self):
        """Test hold_position function"""
        # Set a current position
        self.mock_node.current_status['position'] = [5.0, 3.0, 10.0]
        
        result = self.tools_dict['hold_position']()
        
        # Check that position was published
        self.mock_node.trajectory_pub.publish.assert_called_once()
        
        # Check the message matches current position
        call_args = self.mock_node.trajectory_pub.publish.call_args[0][0]
        self.assertEqual(call_args.position, [5.0, 3.0, -10.0])  # NED frame
        
        self.assertIn("Holding current position", result)
    
    def test_emergency_stop(self):
        """Test emergency_stop function"""
        result = self.tools_dict['emergency_stop']()
        
        # Should call disarm
        self.mock_node.vehicle_command_pub.publish.assert_called_once()
        call_args = self.mock_node.vehicle_command_pub.publish.call_args[0][0]
        self.assertEqual(call_args.param1, 0.0)  # Disarm
        
        self.assertIn("EMERGENCY STOP", result)

if __name__ == '__main__':
    unittest.main()