# naosoccer_pos_action

[![CI](https://github.com/ijnek/naosoccer_pos_action/actions/workflows/main.yml/badge.svg)](https://github.com/ijnek/naosoccer_pos_action/actions/workflows/main.yml)

Executes a .pos file action for a NAO robot, a filetype defined in rUNSWift's codebase.

## Steps

1. Run `rcsoccersim3d`
2. In a new terminal, run `ros2 run rcss3d_agent rcss3d_agent`
3. In a new terminal, run `ros2 run naosoccer_pos_action linear`
4. In a new terminal, publish a start message 
  
    `ros2 topic pub --once start_pos_action std_msgs/msg/Bool '{data: true}'`

Pos file can be specified using rosargs in step 3, such as:

```
ros2 run naosoccer_pos_action linear --ros-args -p "file:=src/naosoccer_pos_action/pos/tilt.pos"
```
