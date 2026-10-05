#!/usr/bin/env python3
"""Check that controller_server actually applied every SE-MPPI plugin key in a params YAML.

ROS 2 silently ignores YAML keys that a node never declares, so a typo, a stale
key or a key the plugin does not read would leave the default in place without
any warning. This script optionally drives controller_server through the
lifecycle `configure` transition (no TF, map or sensors are needed for that),
then fails if any YAML key under the plugin namespace is not declared or holds
a different value.

    ros2 run nav2_controller controller_server --ros-args --params-file P.yaml &
    python3 scripts/check_param_binding.py P.yaml --configure

--allow-unread KEY marks a key that is known to be unread (state why at the
call site); it is reported but does not fail the check.
"""

import argparse
import math
import sys

import rclpy
import yaml
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters


def flatten(prefix, value, out):
    if isinstance(value, dict):
        for k, v in value.items():
            flatten(f'{prefix}.{k}' if prefix else str(k), v, out)
    else:
        out[prefix] = value


def yaml_params(path, node):
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    params = {}
    for name, section in doc.items():  # "/**", a node name or a namespaced one
        if name.strip('/').split('/')[-1] in (node, '**') and 'ros__parameters' in section:
            flatten('', section['ros__parameters'], params)
    return params


def value_of(pv):
    return {
        ParameterType.PARAMETER_BOOL: lambda: pv.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda: pv.integer_value,
        ParameterType.PARAMETER_DOUBLE: lambda: pv.double_value,
        ParameterType.PARAMETER_STRING: lambda: pv.string_value,
        ParameterType.PARAMETER_BOOL_ARRAY: lambda: list(pv.bool_array_value),
        ParameterType.PARAMETER_INTEGER_ARRAY: lambda: list(pv.integer_array_value),
        ParameterType.PARAMETER_DOUBLE_ARRAY: lambda: list(pv.double_array_value),
        ParameterType.PARAMETER_STRING_ARRAY: lambda: list(pv.string_array_value),
    }.get(pv.type, lambda: None)()


def same(expected, actual):
    if isinstance(expected, list) and isinstance(actual, list):
        return len(expected) == len(actual) and all(map(same, expected, actual))
    if isinstance(expected, bool) or isinstance(actual, bool):
        return expected is actual
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        return math.isclose(float(expected), float(actual), rel_tol=0.0, abs_tol=1e-12)
    return expected == actual


def call(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=timeout):
        sys.exit(f'FAIL: service {client.srv_name} not available after {timeout:.0f} s')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        sys.exit(f'FAIL: no response from {client.srv_name}')
    return future.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml')
    ap.add_argument('--node', default='controller_server')
    ap.add_argument('--plugin', default='FollowPath', help='plugin namespace to check')
    ap.add_argument('--configure', action='store_true',
                    help='trigger the lifecycle configure transition first')
    ap.add_argument('--allow-unread', action='append', default=[], metavar='KEY')
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args()

    expected = {k: v for k, v in yaml_params(args.yaml, args.node).items()
                if k.startswith(args.plugin + '.')}
    if not expected:
        sys.exit(f'FAIL: no {args.plugin}.* keys for node {args.node} in {args.yaml}')

    rclpy.init()
    node = rclpy.create_node('se_mppi_param_binding_check')
    target = '/' + args.node.lstrip('/')
    if args.configure:
        state = call(node, node.create_client(GetState, f'{target}/get_state'),
                     GetState.Request(), args.timeout)
        if state.current_state.label == 'unconfigured':
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CONFIGURE
            res = call(node, node.create_client(ChangeState, f'{target}/change_state'),
                       req, args.timeout)
            if not res.success:
                sys.exit(f'FAIL: {target} configure transition failed')
        print(f'{target} configured')

    listed = call(node, node.create_client(ListParameters, f'{target}/list_parameters'),
                  ListParameters.Request(), args.timeout)
    declared = set(listed.result.names)

    undeclared = sorted(k for k in expected if k not in declared)
    keys = sorted(k for k in expected if k in declared)
    req = GetParameters.Request()
    req.names = keys
    got = call(node, node.create_client(GetParameters, f'{target}/get_parameters'),
               req, args.timeout)
    actual = {k: value_of(v) for k, v in zip(keys, got.values)}
    mismatched = [(k, expected[k], actual[k]) for k in keys if not same(expected[k], actual[k])]

    allowed = [k for k in undeclared if k in args.allow_unread]
    failing = [k for k in undeclared if k not in args.allow_unread]
    print(f'{args.yaml}: {len(expected)} {args.plugin}.* YAML keys, '
          f'{len(keys)} declared, {len(keys) - len(mismatched)} hold the YAML value')
    for k in allowed:
        print(f'  known unread (allowed): {k} = {expected[k]!r}')
    for k in failing:
        print(f'  NOT DECLARED by {target}: {k} = {expected[k]!r} (the node ignores it)')
    for k, e, a in mismatched:
        print(f'  MISMATCH {k}: YAML {e!r}, node {a!r}')
    node.destroy_node()
    rclpy.shutdown()
    if failing or mismatched:
        sys.exit('FAIL: parameter binding')
    print('PASS: every plugin key in the YAML is declared and holds the YAML value')


if __name__ == '__main__':
    main()
