import unittest
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from quadrotor_controller.sitl_bridge import SITLBridge, VehicleState




class SITLBridgeTests(unittest.TestCase):
    def test_parse_state_converts_messages_to_vehicle_state(self):
        bridge = SITLBridge()
        state = bridge.parse_state(
            local_position=(1.0, -2.0, 3.0),
            velocity=(0.1, 0.2, -0.3),
            attitude=(0.1, -0.2, 0.3),
            angular_rate=(0.01, -0.02, 0.03),
        )

        self.assertIsInstance(state, VehicleState)
        self.assertEqual(state.position, (1.0, -2.0, 3.0))
        self.assertEqual(state.velocity, (0.1, 0.2, -0.3))
        self.assertEqual(len(state.quaternion), 4)
        self.assertEqual(state.angular_rate, (0.01, -0.02, 0.03))

    def test_build_attitude_target_contains_required_fields(self):
        bridge = SITLBridge()
        message = bridge.build_attitude_target(
            thrust=0.85,
            roll=0.1,
            pitch=-0.05,
            yaw=0.2,
            yaw_rate=0.0,
        )

        self.assertIn("thrust", message)
        self.assertIn("roll", message)
        self.assertIn("pitch", message)
        self.assertIn("yaw", message)
        self.assertEqual(message["thrust"], 0.85)

    def test_connect_sets_connected_flag(self):
        bridge = SITLBridge()
        self.assertFalse(bridge.is_connected())

        bridge.connect()
        self.assertTrue(bridge.is_connected())

        bridge.disconnect()
        self.assertFalse(bridge.is_connected())

    def test_send_attitude_target_requires_connection(self):
        bridge = SITLBridge()
        target = bridge.build_attitude_target(
            thrust=0.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            yaw_rate=0.0,
        )

        with self.assertRaises(RuntimeError):
            bridge.send_attitude_target(target)

        bridge.connect()
        bridge.send_attitude_target(target)
        self.assertEqual(bridge.last_attitude_target, target)


if __name__ == "__main__":
    unittest.main()
