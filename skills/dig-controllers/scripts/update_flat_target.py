#!/usr/bin/env python3
"""Patch an existing flat desired_elevation footprint; never activate or move."""
import argparse
import copy
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from grid_map_msgs.msg import GridMap
from lifecycle_msgs.srv import GetState
from machine_msgs.msg import MachineStatus
from mole_excavation_mapping.srv import ApplySharedExcavationPatch
from rcl_interfaces.srv import GetParameters


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target-z', type=float, required=True)
    p.add_argument('--expected-target-z', type=float, required=True)
    p.add_argument('--expected-cells', type=int, required=True)
    p.add_argument('--output-dir', type=Path, required=True, help='New receipt directory')
    p.add_argument('--controller', default='/mole/dig_ugep_controller')
    p.add_argument('--mapping-node', default='/mole/excavation_mapping')
    p.add_argument('--map-topic', default='/excavation_mapping/grid_map')
    p.add_argument('--patch-service', default='/excavation_mapping/apply_shared_excavation_patch')
    p.add_argument('--footprint-layer', default='runtime_target')
    p.add_argument('--apply', action='store_true', help='Apply once; otherwise save preview only')
    a = p.parse_args()
    if not np.isfinite([a.target_z, a.expected_target_z]).all() or a.expected_cells <= 0:
        p.error('Targets must be finite and expected cell count positive')
    a.output_dir.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    n = rclpy.create_node('update_flat_dig_target')
    seen = {}
    def save(key, msg):
        seen[key] = (msg, time.monotonic())
    subs = [n.create_subscription(kind, topic, lambda msg, key=key: save(key, msg), qos_profile_sensor_data)
            for key, kind, topic in [('map', GridMap, a.map_topic), ('machine', MachineStatus, '/machine_status')]]
    def call(kind, endpoint, request):
        c = n.create_client(kind, endpoint)
        if not c.wait_for_service(timeout_sec=5):
            raise RuntimeError('Unavailable: ' + endpoint)
        f = c.call_async(request)
        rclpy.spin_until_future_complete(n, f, timeout_sec=10)
        if not f.done() or f.result() is None:
            raise RuntimeError('Request timed out; inspect state before retry: ' + endpoint)
        return f.result()
    def geometry(g):
        return (g.header.frame_id, g.info, g.outer_start_index, g.inner_start_index)
    def layers(g):
        return {k: np.asarray(v.data) for k, v in zip(g.layers, g.data)}
    report = {'requested_target_z_m': a.target_z, 'apply_requested': a.apply, 'verified': False}
    try:
        end = time.monotonic() + 8
        while len(seen) < 2 and time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=.1)
        assert len(seen) == 2, 'Missing map or machine status'
        assert call(GetState, a.controller + '/get_state', GetState.Request()).current_state.id == 2, 'Controller must be inactive'
        names = ['use_sim_time', 'desired_elevation_layer_name', 'desired_elevation_offset_m', 'min_valid_elevation_map_z_m', 'depth_shield.tolerance_m']
        params = dict(zip(names, map(parameter_value_to_python, call(GetParameters, a.controller + '/get_parameters', GetParameters.Request(names=names)).values)))
        assert params['use_sim_time'] is False, 'This helper is for hardware with locked acceptance'
        assert params['desired_elevation_layer_name'] == 'desired_elevation' and params['desired_elevation_offset_m'] == 0.0, params
        assert a.target_z - params['depth_shield.tolerance_m'] >= params['min_valid_elevation_map_z_m'], 'Target outside configured elevation validity; review configuration first'
        mask_name = parameter_value_to_python(call(GetParameters, a.mapping_node + '/get_parameters', GetParameters.Request(names=['masked_replace_mask_layer'])).values[0])
        assert mask_name and mask_name != 'desired_elevation', mask_name
        assert all(time.monotonic() - v[1] < 2 for v in seen.values()), 'Stale input'
        machine = seen['machine'][0]
        assert not machine.is_hydraulilock_unlocked and not machine.is_autonomous_operation_unlocked, 'Hydraulics/autonomy acceptance must be locked'
        g = seen['map'][0]
        data = layers(g)
        mask = np.isfinite(data[a.footprint_layer]) & (data[a.footprint_layer] > .5)
        assert int(mask.sum()) == a.expected_cells, 'Unexpected footprint size'
        assert np.allclose(data['desired_elevation'][mask], a.expected_target_z, atol=1e-5, rtol=0), 'Old target mismatch; do not blindly retry'
        if 'dig_zone' in data:
            assert np.all(np.isfinite(data['dig_zone'][mask]) & (data['dig_zone'][mask] > .5)), 'Footprint extends outside dig zone'
        (a.output_dir / 'before-map.cdr').write_bytes(serialize_message(g))
        patch = GridMap(header=copy.deepcopy(g.header), info=copy.deepcopy(g.info),
                        outer_start_index=g.outer_start_index, inner_start_index=g.inner_start_index,
                        layers=['desired_elevation', mask_name], basic_layers=[])
        target = copy.deepcopy(g.data[g.layers.index('desired_elevation')])
        values = np.full(mask.shape, np.nan, dtype=np.float32)
        values[mask] = a.target_z
        target.data = values.tolist()
        patch_mask = copy.deepcopy(target)
        patch_mask.data = mask.astype(np.float32).tolist()
        patch.data = [target, patch_mask]
        (a.output_dir / 'patch.cdr').write_bytes(serialize_message(patch))
        report.update(time=time.time(), old_target_z_m=a.expected_target_z, target_cells=int(mask.sum()),
                      patch_layers=patch.layers, controller_parameters=params, action_sent=False)
        if a.apply:
            response = call(ApplySharedExcavationPatch, a.patch_service, ApplySharedExcavationPatch.Request(patch=patch))
            report['service_response'] = {'success': response.success, 'message': response.message}
            (a.output_dir / 'receipt.json').write_text(json.dumps(report, indent=2) + '\n')
            assert response.success, response.message
            start = time.monotonic()
            while time.monotonic() - start < 8:
                rclpy.spin_once(n, timeout_sec=.1)
                after, received = seen['map']
                if received <= start:
                    continue
                assert geometry(after) == geometry(g), 'Map geometry changed during update'
                current = layers(after)
                assert np.array_equal(current[a.footprint_layer], data[a.footprint_layer], equal_nan=True), 'Footprint changed'
                # Outside authored targets the mapping owner follows live soil.
                # Permit that only where both snapshots show this relationship.
                old_desired, new_desired = data['desired_elevation'], current['desired_elevation']
                unchanged = (old_desired == new_desired) | (np.isnan(old_desired) & np.isnan(new_desired))
                follows_soil = ((old_desired == data['elevation']) | (np.isnan(old_desired) & np.isnan(data['elevation']))) & ((new_desired == current['elevation']) | (np.isnan(new_desired) & np.isnan(current['elevation'])))
                assert np.all((unchanged | follows_soil)[~mask]), 'Unexpected target change outside footprint'
                report['outside_cells_following_live_soil'] = int((~mask & ~unchanged & follows_soil).sum())
                if np.allclose(current['desired_elevation'][mask], a.target_z, atol=1e-5, rtol=0):
                    report.update(verified=True, verified_target_z_m=float(current['desired_elevation'][mask][0]), verified_time=time.time())
                    (a.output_dir / 'after-map.cdr').write_bytes(serialize_message(after))
                    break
            assert report['verified'], 'No matching published target; inspect map before any retry or activation'
    finally:
        (a.output_dir / 'receipt.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
