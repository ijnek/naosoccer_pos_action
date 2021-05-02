# naosoccer_pos_action

Executes a .pos file action for a NAO robot, a filetype defined in rUNSWift's codebase.

## Steps

1. Run `rcsoccersim3d`
2. In a new terminal, run `ros2 launch naosoccer_sim everything_launch.py`
3. In a new terminal, run `ros2 run naosoccer_pos_action linear`
4. In a new terminal, publish a start message 
  
    `ros2 topic pub --once /start_pos_action std_msgs/msg/Bool '{data: true}'`
    
Pos file can be specified using rosargs in step 3.
