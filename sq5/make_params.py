#!/usr/bin/env python3
"""EasyNav parameters for SQ5 in the Kobuki house: the playground's costmap stack and the same
stack on NavMap. Both localize with NavMap's AMCL against the house's Bonxai map, and share the
controller (RPP), the inflation (1 m, scaling 5), the recovery and a 1 Hz planner: only the map
and the planner change.

Usage: make_params.py out_dir   (needs EASYNAV_WS and NAVMAP_DATA)
"""
import copy
import os
import sys

import yaml

WS = os.environ['EASYNAV_WS']
DATA = os.environ['NAVMAP_DATA']
KOBUKI = f'{WS}/src/easynav_playgrounds/playground_kobuki/easynav_playground_kobuki/params/' \
    'costmap.rpp.params.yaml'
SHARE = f'{WS}/install/easynav_playground_kobuki_worlds/share/easynav_playground_kobuki_worlds'

LOCALIZER = {
    'use_sim_time': True,
    'localizer_types': ['amcl'],
    'amcl': {
        'rt_freq': 20.0, 'freq': 5.0, 'reseed_freq': 1.0,
        'plugin': 'easynav_navmap_localizer/AMCLLocalizer',
        'downsampled_cloud_size': 0.1, 'num_particles': 100,
        'noise_translation': 0.1, 'noise_rotation': 0.03, 'noise_translation_to_rotation': 0.02,
        'min_noise_xy': 0.2, 'min_noise_yaw': 0.1, 'compute_odom_from_tf': True,
        'initial_pose': {'x': 0.0, 'y': 0.1, 'yaw': 0.0, 'std_dev_xy': 0.05,
                         'std_dev_yaw': 0.01},
    },
}
BONXAI = {
    'freq': 10.0, 'plugin': 'easynav_bonxai_maps_manager/BonxaiMapsManager',
    'package': 'easynav_playground_tiago_worlds', 'bonxai_path_file': 'maps/house.pcd',
    'resolution': 0.05,
}


def navmap(filters):
    m = {
        'freq': 10.0, 'plugin': 'easynav_navmap_maps_manager/NavMapMapsManager',
        'package': 'easynav_playground_kobuki_worlds',
        'navmap_path_file': os.path.relpath(f'{DATA}/sq5/home2_5cm.navmap', SHARE),
    }
    if filters:
        m['filters'] = ['obstacles', 'inflation']
        m['obstacles'] = {
            'plugin': 'easynav_navmap_maps_manager/NavMapMapsManager/ObstaclesFilter',
            'max_range': 3.0, 'min_height': 0.05, 'max_height': 0.6,
            'downsample_resolution': 0.05,
        }
        m['inflation'] = {
            'plugin': 'easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter',
            'inflation_radius': 1.0, 'cost_scaling_factor': 5.0,
        }
    return m


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    base = yaml.safe_load(open(KOBUKI))
    base['localizer_node']['ros__parameters'] = LOCALIZER
    # Recovery: without the costmap AMCL's own evaluator and mitigation
    rm = base['recovery_node']['ros__parameters']['recovery_manager']
    rm['evaluator_types'] = [e for e in rm['evaluator_types'] if e != 'amcl_convergence']
    rm['mitigation_types'] = [m for m in rm['mitigation_types'] if m != 'amcl_relocalize']
    rm.pop('amcl_convergence', None)
    rm.pop('amcl_relocalize', None)

    costmap = copy.deepcopy(base)
    mm = costmap['maps_manager_node']['ros__parameters']
    mm['map_types'] = ['costmap', 'bonxai', 'navmap']
    mm['bonxai'] = BONXAI
    mm['navmap'] = navmap(filters=False)  # only the localizer's surface
    pl = costmap['planner_node']['ros__parameters']
    pl['simple']['freq'] = 1.0

    nav = copy.deepcopy(base)
    mm = nav['maps_manager_node']['ros__parameters']
    mm.pop('costmap')
    mm['map_types'] = ['bonxai', 'navmap']
    mm['bonxai'] = BONXAI
    mm['navmap'] = navmap(filters=True)
    nav['planner_node']['ros__parameters'] = {
        'use_sim_time': True, 'planner_types': ['astar'],
        'astar': {'freq': 1.0, 'plugin': 'easynav_navmap_planner/AStarPlanner'},
    }
    for name, p in (('costmap', costmap), ('navmap', nav)):
        yaml.safe_dump(p, open(f'{out}/kobuki_{name}.yaml', 'w'), sort_keys=False)
        print(f'{out}/kobuki_{name}.yaml')


if __name__ == '__main__':
    main()
