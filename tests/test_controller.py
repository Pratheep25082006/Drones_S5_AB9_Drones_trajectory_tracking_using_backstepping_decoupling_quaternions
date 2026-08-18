import unittest
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from quadrotor_controller.controller import (


    BacksteppingController,
    ControllerReference,
    ControllerState,
    decompose_attitude,
    normalize_quaternion,
    quaternion_from_euler,
)


class ControllerTests(unittest.TestCase):
    def test_normalize_quaternion(self):
        q = (0.0, 1.0, 0.0, 0.0)
        nq = normalize_quaternion(q)

        norm_sq = nq[0] ** 2 + nq[1] ** 2 + nq[2] ** 2 + nq[3] ** 2
        self.assertAlmostEqual(norm_sq, 1.0, places=6)

    def test_decompose_attitude_recombines_to_original_quaternion(self):
        q = quaternion_from_euler(0.2, -0.1, 0.3)
        q_xy, q_z = decompose_attitude(q)

        self.assertEqual(len(q_xy), 4)
        self.assertEqual(len(q_z), 4)

        # Recombine q_xy and q_z via quaternion multiplication
        q_rec = BacksteppingController._quaternion_multiply(q_xy, q_z)
        for i in range(4):
            self.assertAlmostEqual(q[i], q_rec[i], places=5)


    def test_controller_returns_positive_thrust_and_three_torques(self):
        state = ControllerState(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            quaternion=quaternion_from_euler(0.0, 0.0, 0.0),
            angular_rate=(0.0, 0.0, 0.0),
        )
        reference = ControllerReference(
            position=(1.0, 0.0, 1.0),
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
            heading=0.0,
        )
        controller = BacksteppingController(mass=1.5, gravity=9.81)

        thrust, torques = controller.compute_command(state, reference, dt=0.02)

        self.assertGreater(thrust, 0.0)
        self.assertEqual(len(torques), 3)
        self.assertTrue(all(isinstance(t, float) for t in torques))


if __name__ == "__main__":
    unittest.main()
