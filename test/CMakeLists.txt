project(LottLibsTest)

add_executable(LottLibsTest main.cpp)

# Link against the libraries
# We need cute_lib because main.cpp includes CuteCave.hpp which includes cute.h
target_link_libraries(LottLibsTest PRIVATE 
    Cave 
    CuteCave 
    DistanceMap 
    CuteDistanceMap
    Tracker
    cute_lib
)

# Custom target to ensure DLLs are always copied to the test directory if they change
add_custom_target(CopyLottTestDlls ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:Cave>
        $<TARGET_FILE:CuteCave>
        $<TARGET_FILE:cute>
        $<TARGET_FILE:Tracker>
        $<TARGET_FILE:DistanceMap>
        $<TARGET_FILE:CuteDistanceMap>
        $<TARGET_FILE:Util>
        $<TARGET_FILE_DIR:LottLibsTest>
    VERBATIM
)
add_dependencies(LottLibsTest CopyLottTestDlls)
