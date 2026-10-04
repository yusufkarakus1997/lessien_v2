"""Offline checks: no rclpy.init, node, transport or publish calls."""
import importlib.util
from pathlib import Path
import math
import unittest
from rclpy.serialization import serialize_message, deserialize_message
from lessien_interfaces.msg import WheelCommand

spec = importlib.util.spec_from_file_location('wheel_test_publisher', Path(__file__).with_name('publisher.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class ContractTests(unittest.TestCase):
    def test_fields_and_order(self):
        self.assertEqual(list(WheelCommand.get_fields_and_field_types().items()), m.FIELDS)

    def test_cdr_and_units_order(self):
        # 0 is an arbitrary serialization fixture, NOT a firmware version claim.
        msg = m.make_message(2026100101, 100, 0, [0.1, -0.2, 0.3, -0.4], 1790000000000000)
        wire = serialize_message(msg)
        out = deserialize_message(wire, WheelCommand)
        self.assertEqual(out, msg)
        self.assertEqual(list(out.velocity_rad_s), [0.1, -0.2, 0.3, -0.4])
        self.assertEqual(len(wire), 60)

    def test_sequence_faults_and_recovery(self):
        stream = m.Stream(123, 100, 0, [0.1, 0.2, 0.3, 0.4])
        self.assertEqual(stream.next('normal').sequence, 100)
        self.assertEqual(stream.next('normal').sequence, 101)
        self.assertEqual(stream.next('duplicate').sequence, 101)
        self.assertEqual(stream.next('duplicate').sequence, 101)
        self.assertEqual(stream.next('old').sequence, 100)
        self.assertEqual(stream.next('normal').sequence, 102)

    def test_uint32_wrap(self):
        stream = m.Stream(123, 0xffffffff, 0, [0.0]*4)
        self.assertEqual(stream.next('normal').sequence, 0xffffffff)
        self.assertEqual(stream.next('normal').sequence, 0)
        self.assertEqual(stream.next('old').sequence, 0xffffffff)
        self.assertEqual(stream.next('normal').sequence, 1)

    def test_nonfinite_every_wheel_roundtrip(self):
        stream = m.Stream(123, 100, 0, [0.1, 0.2, 0.3, 0.4])
        for kind in ['nan', 'inf', 'neg-inf']:
            for index in range(4):
                out = deserialize_message(serialize_message(stream.next(kind, index)), WheelCommand)
                for i, v in enumerate(out.velocity_rad_s):
                    self.assertEqual(math.isfinite(v), i != index)
                if kind == 'nan':
                    self.assertTrue(math.isnan(out.velocity_rad_s[index]))
                else:
                    self.assertEqual(out.velocity_rad_s[index], math.inf if kind == 'inf' else -math.inf)

    def test_suite_silent_and_recovery_phases(self):
        phases = list(m.phases('suite'))
        self.assertEqual(phases[-1][0], 'stale')
        self.assertEqual(sum(k == 'stale' for k, _ in phases), 2)
        self.assertEqual(sum(k == 'nan' for k, _ in phases), 4)
        for i, (kind, _) in enumerate(phases):
            if kind in ['duplicate', 'old', 'nan', 'inf', 'neg-inf']:
                self.assertEqual(phases[i-1][0], 'normal')


if __name__ == '__main__':
    unittest.main(verbosity=2)
