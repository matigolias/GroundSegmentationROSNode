find_package(catkin REQUIRED COMPONENTS cv_bridge)

add_library(labeler SHARED label.cpp)
target_include_directories(labeler PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(labeler ${catkin_LIBRARIES})
            

