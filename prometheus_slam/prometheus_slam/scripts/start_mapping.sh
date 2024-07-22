#!/usr/bin/env sh
rosservice call /prometheus_slam/tsdf_client_0/toggle_mapping "data: True"
rosservice call /prometheus_slam/tsdf_client_1/toggle_mapping "data: True"
rosservice call /prometheus_slam/tsdf_client_2/toggle_mapping "data: True"
