# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

add_subdirectory(actuator)

if(AIMRT_MUJOCO_SIM_BUILD_EXAMPLES)
  add_subdirectory(examples/inverted_pendulum)
endif()
