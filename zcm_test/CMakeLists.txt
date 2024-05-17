project(zcm_test)

aux_source_directory(./ SRC_LIST)

add_executable(zcm_sub Sub.cpp)
add_executable(zcm_pub Pub.cpp)
add_executable(zcm_traj_pub ZcmTrajectoryPublisher.cpp)

target_link_libraries(zcm_sub zcm pthread)
target_link_libraries(zcm_pub zcm)
target_link_libraries(zcm_traj_pub zcm)