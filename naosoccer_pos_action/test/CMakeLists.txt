# Build test_parser
ament_add_gtest(test_parser
  test_parser.cpp)

target_link_libraries(test_parser
  naosoccer_pos_action_node
)
