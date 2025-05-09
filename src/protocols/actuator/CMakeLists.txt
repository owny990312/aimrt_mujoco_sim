# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)

set(CUR_SUPERIOR_NAMESPACE "aimrt::protocols")
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Add target
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME ${CUR_TARGET_NAME}_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS aimrt::protocols::common_pb_gencode)
add_library(${CUR_TARGET_ALIAS_NAME}_pb_gencode ALIAS ${CUR_TARGET_NAME}_pb_gencode)

# Set installation
if(AIMRT_INSTALL)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DESTINATION "share"
    FILES_MATCHING
    PATTERN "*.proto")

  set_property(TARGET ${CUR_TARGET_NAME}_pb_gencode PROPERTY EXPORT_NAME ${CUR_TARGET_ALIAS_NAME}_pb_gencode)
  install(
    TARGETS ${CUR_TARGET_NAME}_pb_gencode
    EXPORT ${INSTALL_CONFIG_NAME}
    ARCHIVE DESTINATION lib
            FILE_SET HEADERS
            DESTINATION include/${CUR_TARGET_NAME}_pb_gencode)
endif()
