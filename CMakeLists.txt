cmake_minimum_required(VERSION 3.20)
project(Lott-LIBS)


# --- 1. Global Options & Flags ---
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# This flag tells submodules (Maze, Cave, etc.) to SKIP their own dependency fetching
# because Lott-Libs is handling it globally.
set(ALL_SUBS_BUILD_ON ON)

option(BUILD_GODOT "Build with Godot support" ON)
option(BUILD_CUTE "Build with Cute Framework support" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

include(FetchContent)

# --- 2. Dependencies: Godot (FetchContent) ---
if(BUILD_GODOT)
    FetchContent_Declare(
            GDExtension
            GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
            GIT_TAG godot-4.3-stable
    )
endif()

# --- 3. Dependencies: Common Libs (FetchContent) ---
FetchContent_Declare(
    Libs
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Libs"
    # GIT_REPOSITORY https://github.com/TamToucan/Libs.git
    # GIT_TAG main
)

# Make dependencies available globally
if(BUILD_GODOT)
    FetchContent_MakeAvailable(GDExtension Libs)
else()
    FetchContent_MakeAvailable(Libs)
endif()

# --- 4. Dependencies: Cute Framework (Bypassable for Submodule use) ---
if(BUILD_CUTE)
    # Check if 'cute' target was already defined by a parent project (like CuteLott)
    if(TARGET cute)
        message(STATUS "[Lott-Libs] Using Cute Framework target provided by parent project.")
        # Create the alias 'cute_lib' that the submodules expect
        if(NOT TARGET cute_lib)
            add_library(cute_lib INTERFACE GLOBAL)
            target_link_libraries(cute_lib INTERFACE cute)
        endif()
        set(CUTE_FOUND ON)
    else()
        # Fallback to local detection if Lott-Libs is built standalone
        set(CUTE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deps/cute")
        
        # Check for BOTH static (.a) and DLL import (.dll.a) for MinGW flexibility
        if(EXISTS "${CUTE_ROOT}/lib/libcute.a" OR EXISTS "${CUTE_ROOT}/lib/libcute.dll.a")
            message(STATUS "[Lott-Libs] Manual Cute Framework detection: FOUND at ${CUTE_ROOT}")
            
            add_library(cute UNKNOWN IMPORTED GLOBAL)
            
            # Detect which file exists to support both static and DLL
            if(EXISTS "${CUTE_ROOT}/lib/libcute.dll.a")
                set_target_properties(cute PROPERTIES
                    IMPORTED_IMPLIB "${CUTE_ROOT}/lib/libcute.dll.a"
                    IMPORTED_LOCATION "${CUTE_ROOT}/bin/libcute.dll"
                )
            else()
                set_target_properties(cute PROPERTIES
                    IMPORTED_LOCATION "${CUTE_ROOT}/lib/libcute.a"
                )
            endif()

            set_target_properties(cute PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${CUTE_ROOT}/include;${CUTE_ROOT}"
            )
            add_library(cute_lib ALIAS cute)
            set(CUTE_FOUND ON)
        else()
            message(WARNING "[Lott-Libs] Cute Framework NOT FOUND in deps/cute. Disabling BUILD_CUTE.")
            set(BUILD_CUTE OFF)
        endif()
    endif()
endif()

# --- 5. Add Submodules ---
# These directories must contain their own CMakeLists.txt (see Maze-LIB below)
add_subdirectory(Tracker-LIB)
add_subdirectory(Maze-LIB)
add_subdirectory(Cave-LIB)
add_subdirectory(DistanceMap-LIB)

# Test suite (if you have a top-level test folder)
add_subdirectory(test)

# --- 6. Custom Aggregation Targets ---
# These targets group the submodule targets for easy building

# A. CORE (Pure C++)
add_custom_target(core)
# Depends on the specific library targets defined in submodules
add_dependencies(core TrackerLib::Tracker Maze Cave DistanceMap)

# B. GODOT (Extension)
if(BUILD_GODOT)
    add_custom_target(myGodot)
    add_dependencies(myGodot GDDistanceMap GDTracker GDMaze GDCave core)
endif()

# C. CUTE (New Framework)
if(BUILD_CUTE)
    add_custom_target(myCute)
    add_dependencies(myCute CuteDistanceMap CuteTracker CuteMaze CuteCave core)
endif()
