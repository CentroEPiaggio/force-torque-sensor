cmake_minimum_required(VERSION 2.8)

include_directories (
  include
)

add_library(FTSensor SHARED
  src/FTSensor.cpp
)

target_link_libraries(FTSensor
  ${LIBXML2_LIBRARIES}
)

install(TARGETS FTSensor EXPORT FTSensorTargets LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT libraries)