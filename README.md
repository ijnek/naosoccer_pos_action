# naosoccer_pos_action

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_humble.yaml?query=branch:iron)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_iron.yaml?query=branch:iron)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

Executes a .pos file action for a NAO robot, a filetype defined in rUNSWift's codebase.

## Steps

1. Run `rcsoccersim3d`
2. In a new terminal, run `ros2 run rcss3d_nao rcss3d_nao`
3. In a new terminal, run `ros2 run naosoccer_pos_action naosoccer_pos_action`
4. In a new terminal, publish a start message

    `ros2 topic pub --once start_pos_action std_msgs/msg/Bool '{data: true}'`

Pos file can be specified using rosargs in step 3, such as:

```
ros2 run naosoccer_pos_action linear --ros-args -p "file:=src/naosoccer_pos_action/pos/tilt.pos"
```
