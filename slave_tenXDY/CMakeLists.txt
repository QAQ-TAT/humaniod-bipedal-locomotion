
set(SOURCES slave_tenXDY.c)
add_executable(slave_tenXDY ${SOURCES})
target_link_libraries(slave_tenXDY soem)
install(TARGETS slave_tenXDY DESTINATION bin)
