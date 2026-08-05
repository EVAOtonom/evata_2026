# right_side_bias_layer

ROS 2 Humble / Nav2 Costmap2D plugin that adds a continuous asymmetric cost field around a CSV centerline.

## Build

```bash
cd ~/real_ws/src
unzip right_side_bias_layer.zip
cd ~/real_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select right_side_bias_layer
source install/setup.bash
```

## Important direction convention

The CSV points must be ordered in the intended driving direction. Relative to that direction, the geometric right side receives the preferred cost. For travel in the reverse direction, set `invert_side: true` or provide a reversed centerline.

## Verify registration

```bash
ros2 plugin list | grep right_side_bias_layer
```

## Files

- `config/centerline.csv`: centerline from `(-1.52, 0.5)` to `(-4.10, -14.50)`, uniformly sampled.
- `config/nav2_layer_example.yaml`: example integration snippet.
