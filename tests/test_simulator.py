import unittest

from simulator import MultiRateSimulator


class SimulatorTests(unittest.TestCase):
    def test_rate_validation(self):
        with self.assertRaises(ValueError):
            MultiRateSimulator(10, 20, 5)

    def test_sensor_count(self):
        sim = MultiRateSimulator(100, 20, 10)
        sim.run(2.0)
        self.assertEqual(len(sim.telemetry), 20)

    def test_deterministic_run(self):
        a = MultiRateSimulator()
        b = MultiRateSimulator()
        a.run(4.0)
        b.run(4.0)
        self.assertEqual(a.summary(), b.summary())

    def test_controller_clamps(self):
        sim = MultiRateSimulator()
        sim.command.speed_mps = 1000
        sim._controller_step()
        self.assertEqual(sim.control["throttle"], 1.0)


if __name__ == "__main__":
    unittest.main()
