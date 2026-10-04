#!/usr/bin/env python3
"""Headless integration test. Publishes only to the fixed mock command topic.
Requires ROS_DOMAIN_ID=77 and verifies GenericSystem before creating a publisher.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, ListHardwareInterfaces
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

NS = '/lessien_mock'
JOINTS = [f'{corner}_wheel_joint' for corner in
          ('front_left', 'rear_left', 'front_right', 'rear_right')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='mock_checks/runtime.json')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        raise SystemExit('Refusing to publish: run in dedicated ROS_DOMAIN_ID=77')
    rclpy.init()
    node = rclpy.create_node('verify_mock', namespace=NS)
    data = {'joints': [], 'odom': [], 'tf': {}, 'description': None}
    report = {'scope': 'MOCK ONLY; not hardware validation', 'domain': 77}
    publisher = None

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def call(service, srv_type, request):
        client = node.create_client(srv_type, service)
        assert client.wait_for_service(timeout_sec=20), service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        assert future.done() and future.result() is not None, service
        result = future.result()
        node.destroy_client(client)
        return result

    def tf_callback(msg):
        for transform in msg.transforms:
            data['tf'][transform.child_frame_id] = transform

    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [
        node.create_subscription(JointState, NS + '/joint_states',
            lambda m: data['joints'].append((time.monotonic(), m)), 100),
        node.create_subscription(Odometry, NS + '/diff_drive_controller/odom',
            lambda m: data['odom'].append((time.monotonic(), m)), 100),
        node.create_subscription(TFMessage, NS + '/tf', tf_callback, 100),
        node.create_subscription(TFMessage, NS + '/tf_static', tf_callback, transient),
        node.create_subscription(String, NS + '/robot_description',
            lambda m: data.update(description=m.data), transient),
    ]
    try:
        hardware = call(NS + '/controller_manager/list_hardware_components',
                        ListHardwareComponents, ListHardwareComponents.Request()).component
        assert len(hardware) == 1
        assert hardware[0].plugin_name == 'mock_components/GenericSystem'
        assert hardware[0].state.label == 'active'
        report['hardware_plugin'] = hardware[0].plugin_name
        controllers = call(NS + '/controller_manager/list_controllers',
                           ListControllers, ListControllers.Request()).controller
        assert {c.name for c in controllers} == {'joint_state_broadcaster', 'diff_drive_controller'}
        assert all(c.state == 'active' for c in controllers)
        report['controllers'] = {c.name: c.state for c in controllers}
        interfaces = call(NS + '/controller_manager/list_hardware_interfaces',
                         ListHardwareInterfaces, ListHardwareInterfaces.Request())
        commands = {i.name: i for i in interfaces.command_interfaces}
        states = {i.name: i for i in interfaces.state_interfaces}
        for j in JOINTS:
            assert commands[j + '/velocity'].is_available and commands[j + '/velocity'].is_claimed
            assert states[j + '/velocity'].is_available and states[j + '/position'].is_available
        report['command_interfaces'] = sorted(commands)
        report['state_interfaces'] = sorted(states)
        request = GetParameters.Request(names=['wheel_radius', 'wheel_separation',
                                               'position_feedback', 'open_loop', 'cmd_vel_timeout'])
        values = call(NS + '/diff_drive_controller/get_parameters', GetParameters, request).values
        radius, separation, feedback, open_loop, timeout = (
            values[0].double_value, values[1].double_value, values[2].bool_value,
            values[3].bool_value, values[4].double_value)
        assert radius > 0 and separation > 0 and not feedback and not open_loop and timeout > 0
        report['parameters'] = dict(zip(request.names, [radius, separation, feedback, open_loop, timeout]))
        # Discovery of transient-local description can take longer than one second.
        deadline = time.monotonic() + 10.0
        while not (data['description'] and data['joints'] and data['odom']) and time.monotonic() < deadline:
            spin(0.1)
        assert data['description'] and data['joints'] and data['odom'], 'Model/state topics not ready within 10 s'
        root = ET.fromstring(data['description'])
        assert [p.text for p in root.findall('./ros2_control/hardware/plugin')] == ['mock_components/GenericSystem']
        assert root.find('./ros2_control/hardware/param[@name="calculate_dynamics"]').text == 'true'
        joints = {j.attrib['name']: j for j in root.findall('joint')}
        positions = {}
        for j in JOINTS:
            assert joints[j].attrib['type'] == 'continuous'
            assert joints[j].find('axis').attrib['xyz'] == '0 1 0'
            positions[j] = list(map(float, joints[j].find('origin').attrib['xyz'].split()))
        assert abs(positions[JOINTS[0]][1] - positions[JOINTS[2]][1] - separation) < 1e-9
        for name in JOINTS:
            r = root.find(f'./link[@name="{name.replace("joint", "link")}"]/visual/geometry/cylinder')
            assert abs(float(r.attrib['radius']) - radius) < 1e-9
        topic = NS + '/diff_drive_controller/cmd_vel'
        subscribers = node.get_subscriptions_info_by_topic(topic)
        assert len(subscribers) == 1
        assert subscribers[0].node_name == 'diff_drive_controller'
        assert subscribers[0].node_namespace == NS
        assert subscribers[0].topic_type == 'geometry_msgs/msg/TwistStamped'
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        report['command_topic'] = topic
        report['command_type'] = subscribers[0].topic_type
        # Only create the publisher after checking mock hardware and its subscriber.
        publisher = node.create_publisher(TwistStamped, topic, 10)
        spin(0.4)

        last_publish_time = None

        def command(v, w):
            nonlocal last_publish_time
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'lessien_mock/base_footprint'
            msg.twist.linear.x = v
            msg.twist.angular.z = w
            last_publish_time = time.monotonic()
            publisher.publish(msg)

        def phase(name, v, w):
            start = time.monotonic()
            initial_position = dict(zip(data['joints'][-1][1].name, data['joints'][-1][1].position))
            while time.monotonic() - start < 1.4:
                command(v, w)
                spin(0.05)
            samples = [m for t, m in data['joints'] if t > start + 0.6]
            expected = [(v - w*separation/2)/radius]*2 + [(v + w*separation/2)/radius]*2
            assert len(samples) >= 10
            for sample in samples:
                observed = dict(zip(sample.name, sample.velocity))
                assert set(sample.name) == set(JOINTS)
                for j, target in zip(JOINTS, expected):
                    assert abs(observed[j] - target) < 0.03, (name, j, observed[j], target)
            final = samples[-1]
            final_pos = dict(zip(final.name, final.position))
            for j, target in zip(JOINTS, expected):
                assert (final_pos[j] - initial_position[j]) * target > 0.01
            odom = [m for t, m in data['odom'] if t > start + 0.6]
            assert len(odom) >= 10
            assert all(abs(m.twist.twist.linear.x-v) < 0.01 and
                       abs(m.twist.twist.angular.z-w) < 0.01 for m in odom)
            assert odom[-1].header.frame_id == 'lessien_mock/odom'
            assert odom[-1].child_frame_id == 'lessien_mock/base_footprint'
            report[name] = {'result': 'PASS', 'expected_rad_s': expected,
                            'observed_rad_s': dict(zip(final.name, final.velocity)),
                            'joint_samples': len(samples), 'odom_samples': len(odom)}

        phase('forward', 0.1, 0.0)
        phase('turn_left_in_place', 0.0, 0.5)
        # No stop command here: verify the controller's own watchdog.
        stopped_publishing = last_publish_time
        spin(timeout + 0.7)
        samples = [(t, m) for t, m in data['joints'] if t > stopped_publishing]
        zero = [(t, m) for t, m in samples if all(abs(v) < 1e-6 for v in m.velocity)]
        assert zero and zero[0][0] - stopped_publishing < timeout + 0.25
        assert all(abs(v) < 1e-6 for t, m in samples if t > stopped_publishing + timeout + 0.2 for v in m.velocity)
        report['command_timeout'] = {'result': 'PASS', 'configured_s': timeout,
                                    'observed_stop_after_last_publish_s': zero[0][0]-stopped_publishing}
        expected_edges = {'lessien_mock/base_footprint': 'lessien_mock/odom',
                          'lessien_mock/base_link': 'lessien_mock/base_footprint',
                          **{'lessien_mock/' + j.replace('joint', 'link'): 'lessien_mock/base_link' for j in JOINTS}}
        for child, parent in expected_edges.items():
            assert child in data['tf'] and data['tf'][child].header.frame_id == parent
        base_z = float(joints['base_footprint_joint'].find('origin').attrib['xyz'].split()[2])
        assert abs(data['tf']['lessien_mock/base_link'].transform.translation.z-base_z) < 1e-9
        final = data['joints'][-1][1]
        final_positions = dict(zip(final.name, final.position))
        for j in JOINTS:
            transform = data['tf']['lessien_mock/' + j.replace('joint', 'link')].transform
            observed = (transform.translation.x, transform.translation.y, transform.translation.z)
            assert all(abs(a-b) < 1e-9 for a,b in zip(observed, positions[j]))
            assert abs(base_z + transform.translation.z - radius) < 1e-9
            q = transform.rotation
            assert abs(q.x) < 1e-6 and abs(q.z) < 1e-6
            assert abs(abs(q.y*math.sin(final_positions[j]/2) + q.w*math.cos(final_positions[j]/2))-1) < 1e-5
        report['tf'] = {'result': 'PASS', 'edges': expected_edges,
                        'checks': 'geometry translations, ground contact, +Y rotation agrees with joint positions'}
        report['real_cmd_vel_publishers_in_test_domain'] = len(node.get_publishers_info_by_topic('/cmd_vel'))
        assert report['real_cmd_vel_publishers_in_test_domain'] == 0
        report['result'] = 'PASS'
    except Exception as error:
        report['result'] = 'FAIL'
        report['error'] = repr(error)
        raise
    finally:
        if publisher is not None:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(msg)  # fixed mock topic only
            spin(0.1)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
