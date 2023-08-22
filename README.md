# naosoccer_pos_action

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_humble.yaml?query=branch:rolling)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_iron.yaml?query=branch:rolling)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

Executes a .pos file action for a NAO robot, a filetype defined in rUNSWift's codebase.

## Steps (on real robot)

1. On a terminal on the robot, run `ros2 run nao_lola nao_lola`
2. In a new terminal (either on robot, or on your machine), run `ros2 run naosoccer_pos_action naosoccer_pos_action`
3. In a new terminal (either on robot, or on your machine), publish a start message

    `ros2 topic pub --once start_pos_action std_msgs/msg/Bool '{data: true}'`

## Steps (simulation using rcsoccer3d)

1. Run `rcsoccersim3d`
2. In a new terminal, run `ros2 run rcss3d_nao rcss3d_nao`
3. In a new terminal, run `ros2 run naosoccer_pos_action naosoccer_pos_action`
4. In a new terminal, publish a start message

    `ros2 topic pub --once start_pos_action std_msgs/msg/Bool '{data: true}'`

## Using a different motion.

Pos files define the different motions, and can be specified using a ros parameter for the naosoccer_pos_action node.
To set the parameter, when running the naosoccer_pos_action node in the steps above, instead do the following:

```
ros2 run naosoccer_pos_action linear --ros-args -p "file:=src/naosoccer_pos_action/pos/tilt.pos"
```
