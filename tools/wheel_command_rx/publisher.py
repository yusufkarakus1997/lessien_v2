#!/usr/bin/env python3
"""WheelCommand test generator. Default is offline; --publish explicitly enables DDS.
Protocol version and topic verified from the transferred firmware source/README.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time

HASH = 'cb0405a45866b3cc63c502e0e726fe0fb00bdeaa425f7eb4874accd595472d6b'
ROOT = Path(__file__).resolve().parents[2]
FIELDS = [('source_session_id', 'uint64'), ('sample_time_unix_us', 'uint64'),
          ('sequence', 'uint32'), ('protocol_version', 'uint32'),
          ('velocity_rad_s', 'double[4]')]


def uint(bits):
    def parse(value):
        number = int(value, 0)
        if not 0 <= number < (1 << bits):
            raise argparse.ArgumentTypeError(f'expected uint{bits}')
        return number
    return parse


def make_message(session, seq, version, values, stamp=None):
    from lessien_interfaces.msg import WheelCommand
    msg = WheelCommand()
    msg.source_session_id = session
    msg.sample_time_unix_us = time.time_ns() // 1000 if stamp is None else stamp
    msg.sequence = seq
    msg.protocol_version = version
    msg.velocity_rad_s = values
    return msg


def phases(case):
    if case == 'suite':
        yield ('normal', None)
        yield ('stale', None)
        for kind in ('duplicate', 'old', 'nan', 'inf', 'neg-inf'):
            for index in (range(4) if kind in ('nan', 'inf', 'neg-inf') else [None]):
                yield ('normal', None)
                yield (kind, index)
        yield ('normal', None)
        yield ('stale', None)
    else:
        if case != 'normal':
            yield ('normal', None)
        yield (case, None)


class Stream:
    def __init__(self, session, sequence, version, values):
        self.session, self.sequence, self.version = session, sequence, version
        self.values = values
        self.last_normal = None
        self.last_stamp = 0

    def next(self, kind, index=0):
        seq = self.sequence
        values = list(self.values)
        if kind in ('duplicate', 'old'):
            if self.last_normal is None:
                raise ValueError('normal baseline required')
            seq = self.last_normal if kind == 'duplicate' else (self.last_normal - 1) % (1 << 32)
        else:
            self.sequence = (self.sequence + 1) % (1 << 32)
            if kind == 'normal':
                self.last_normal = seq
            elif kind in ('nan', 'inf', 'neg-inf'):
                values[index] = {'nan': math.nan, 'inf': math.inf, 'neg-inf': -math.inf}[kind]
            else:
                raise ValueError(kind)
        # Fresh source time isolates duplicate/old sequence tests from source-age tests.
        stamp = time.time_ns() // 1000
        if stamp < self.last_stamp:
            raise RuntimeError('system clock moved backwards; stop instead of fabricating a timestamp')
        self.last_stamp = stamp
        return make_message(self.session, seq, self.version, values, stamp)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol-version', type=uint(32), default=1, help='firmware WHEEL_COMMAND_PROTOCOL_VERSION=1; offline fixtures may override')
    parser.add_argument('--session-id', type=uint(64), default=2026100101,
                        help='fixed test ID; constant throughout every phase')
    parser.add_argument('--start-sequence', type=uint(32), default=100)
    parser.add_argument('--values', type=float, nargs=4, default=[0.1, 0.2, 0.3, 0.4],
                        metavar=('FL', 'RL', 'FR', 'RR'))
    parser.add_argument('--case', choices=['normal', 'stale', 'duplicate', 'old', 'nan', 'inf', 'neg-inf', 'suite'], default='suite')
    parser.add_argument('--wheel-index', type=int, choices=range(4), default=0)
    parser.add_argument('--seconds', type=float, default=20.0, help='seconds per publishing phase at 20 Hz')
    parser.add_argument('--stale-seconds', type=float, default=20.0, help='silent interval; no final zero message')
    parser.add_argument('--publish', action='store_true', help='explicitly enable domain 0 DDS publication')
    parser.add_argument('--topic', default='/pico_b/wheel_commands', help='fixed topic confirmed by laptop README')
    parser.add_argument('--firmware-contract', type=Path,
                        help='matching laptop README; required for live mode after manual contract review')
    parser.add_argument('--log', type=Path, default=Path('wheel_command_checks/publisher.jsonl'))
    args = parser.parse_args()
    if not args.session_id or not all(math.isfinite(v) for v in args.values):
        parser.error('nonzero session ID and four finite baseline values required')
    if not all(math.isfinite(v) and 0 < v <= 300 for v in (args.seconds, args.stale_seconds)):
        parser.error('phase durations must be finite, >0 and <=300 seconds')
    source = ROOT / 'src/lessien_interfaces/msg/WheelCommand.msg'
    if hashlib.sha256(source.read_bytes()).hexdigest() != HASH:
        parser.error('WheelCommand.msg SHA256 mismatch')
    from lessien_interfaces.msg import WheelCommand
    from rclpy.serialization import serialize_message, deserialize_message
    if list(WheelCommand.get_fields_and_field_types().items()) != FIELDS:
        parser.error('installed WheelCommand fields/order mismatch; source correct overlay')
    if args.publish:
        if os.environ.get('ROS_DOMAIN_ID') != '0':
            parser.error('--publish requires explicit ROS_DOMAIN_ID=0')
        contract = json.loads((ROOT / 'wheel_command_checks/contract.json').read_text())
        if not contract['ready_for_live_test']:
            parser.error('live mode blocked: ' + contract['reason'])
        if args.topic != contract['topic'] or args.protocol_version != contract['protocol_version']:
            parser.error('topic/protocol version differs from verified firmware contract')
        if not args.firmware_contract or not args.firmware_contract.is_file():
            parser.error('matching firmware README required')
        if hashlib.sha256(args.firmware_contract.read_bytes()).hexdigest() != contract['firmware_readme_sha256']:
            parser.error('firmware README hash changed; contract review required')
        firmware = ROOT / 'wheel_command_checks/laptop_pico_b_node_app.cpp'
        if hashlib.sha256(firmware.read_bytes()).hexdigest() != contract['firmware_source_sha256']:
            parser.error('firmware source hash changed; contract review required')
    if args.log.exists():
        parser.error('log already exists; choose a new --log path')
    args.log.parent.mkdir(parents=True, exist_ok=True)
    node = publisher = None
    try:
        if args.publish:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
            rclpy.init(args=[])
            node = rclpy.create_node('wheel_command_rx_test')
            # No remapping arguments are accepted. Never publish to /cmd_vel.
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_publishers_info_by_topic(args.topic):
                raise RuntimeError('another publisher already exists on the selected topic')
            subscribers = node.get_subscriptions_info_by_topic(args.topic)
            if len(subscribers) != 1 or subscribers[0].node_name != 'pico_b_control_node' or subscribers[0].topic_type != 'lessien_interfaces/msg/WheelCommand':
                raise RuntimeError('expected exactly one Pico B WheelCommand subscriber')
            publisher = node.create_publisher(WheelCommand, args.topic, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
        stream = Stream(args.session_id, args.start_sequence, args.protocol_version, args.values)
        count = 0
        with args.log.open('x') as log:
            def record(data):
                log.write(json.dumps(data, allow_nan=False) + '\n')
                log.flush()
            record({'mode': 'LIVE' if args.publish else 'OFFLINE_NO_DDS', 'msg_sha256': HASH,
                    'session': args.session_id, 'version': args.protocol_version, 'hz': 20,
                    'topic': args.topic, 'source_time_mapping_verified': False,
                    'firmware_readme_sha256': hashlib.sha256(args.firmware_contract.read_bytes()).hexdigest() if args.firmware_contract else None})
            for kind, index in phases(args.case):
                print(f'{kind} wheel={index}: {"LIVE" if publisher else "offline"}', flush=True)
                record({'event': 'phase_start', 'phase': kind, 'wheel_index': index, 'monotonic_ns': time.monotonic_ns()})
                if kind == 'stale':
                    # Offline records intent without waiting. Live sends absolutely nothing.
                    if publisher:
                        end = time.monotonic() + args.stale_seconds
                        while time.monotonic() < end:
                            rclpy.spin_once(node, timeout_sec=min(0.05, max(0, end-time.monotonic())))
                    record({'event': 'silent_interval', 'seconds': args.stale_seconds})
                    continue
                samples = max(1, math.ceil(args.seconds * 20))
                next_send = time.monotonic()
                for _ in range(samples):
                    if publisher:
                        time.sleep(max(0, next_send - time.monotonic()))
                    msg = stream.next(kind, args.wheel_index if index is None else index)
                    wire = serialize_message(msg)
                    decoded = deserialize_message(wire, WheelCommand)
                    if publisher:
                        if node.count_subscribers(args.topic) != 1 or node.count_publishers(args.topic) != 1:
                            raise RuntimeError('topic endpoint count changed; stopping')
                        publisher.publish(msg)
                    record({'phase': kind, 'sequence': decoded.sequence,
                            'sample_time_unix_us': decoded.sample_time_unix_us,
                            'velocity_rad_s': [v if math.isfinite(v) else str(v) for v in decoded.velocity_rad_s],
                            'cdr_bytes': len(wire), 'sent': publisher is not None,
                            'monotonic_ns': time.monotonic_ns()})
                    count += 1
                    # Avoid catch-up bursts after scheduler delays.
                    next_send = max(next_send + 0.05, time.monotonic() + 0.001)
            record({'event': 'complete', 'messages': count, 'published': publisher is not None})
        print(f'{count} messages; {"published" if publisher else "serialized only; no ROS node created"}')
    except KeyboardInterrupt:
        print('Stopped; no final zero/keepalive message sent (stale test).')
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
