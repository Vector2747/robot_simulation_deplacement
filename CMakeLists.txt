
cmake_minimum_required(VERSION 3.20)

project(RobotNavigation LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================
# RAYLIB
# ============================================

set(RAYLIB_DIR "$ENV{USERPROFILE}/source/raylib-msvc")

# ============================================
# DEAR IMGUI + RLIMGUI
# ============================================

set(RLIMGUI_DIR "${CMAKE_SOURCE_DIR}/external/rlImGui")

file(GLOB IMGUI_SOURCES CONFIGURE_DEPENDS
    "${RLIMGUI_DIR}/imgui/*.cpp"
)

# ============================================
# ROBOT NAVIGATION - SOURCES
# ============================================

add_executable(RobotNavigation

    # Main
    src/main.cpp

    # World
    src/monde/Monde.cpp
    src/monde/EvilAhrEvent.cpp

    # Robot and target
    src/robot/Robot.cpp
    src/cible/Cible.cpp

    # Input
    src/input/CameraCtrl.cpp
    src/input/ManualControlleur.cpp

    # Physics
    src/physics/CollisionChecker.cpp

    # Planning
    src/planification/GridMap.cpp
    src/planification/DStarLite.cpp
    src/planification/GridSynchronizer.cpp

    # Render
    src/rendering/WorldRender.cpp

    # Interface
    src/interface/Dashboard.cpp

    # Navigation
    src/navigation/NavigationController.cpp
    src/navigation/TemporalAStar.cpp
    src/navigation/LocalPlanner.cpp
    src/navigation/PathFollower.cpp
    src/navigation/LocalGrid.cpp

    # Dear ImGui
    "${RLIMGUI_DIR}/rlImGui.cpp"
    ${IMGUI_SOURCES}
)

# ============================================
# INCLUDE DIRECTORIES
# ============================================

target_include_directories(RobotNavigation PRIVATE

    "${CMAKE_SOURCE_DIR}/src"

    "${RAYLIB_DIR}/src"

    "${RLIMGUI_DIR}"
    "${RLIMGUI_DIR}/imgui"
)

# ============================================
# LINK LIBRARIES
# ============================================

target_link_libraries(RobotNavigation PRIVATE

    "${RAYLIB_DIR}/build-msvc/raylib/Release/raylib.lib"

    opengl32
    gdi32
    winmm
)

include(CTest)

if(BUILD_TESTING)
    set(NAVIGATION_TEST_SUPPORT_SOURCES
        src/monde/Monde.cpp
        src/monde/EvilAhrEvent.cpp
        src/robot/Robot.cpp
        src/cible/Cible.cpp
        src/physics/CollisionChecker.cpp
        src/planification/GridMap.cpp
        src/planification/DStarLite.cpp
        src/planification/GridSynchronizer.cpp
        src/navigation/NavigationController.cpp
        src/navigation/TemporalAStar.cpp
        src/navigation/LocalPlanner.cpp
        src/navigation/PathFollower.cpp
        src/navigation/LocalGrid.cpp
    )

    add_executable(
        RobotNavigationTemporalTests
        tests/TemporalNavigationTests.cpp
        ${NAVIGATION_TEST_SUPPORT_SOURCES}
    )

    target_include_directories(RobotNavigationTemporalTests PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${RAYLIB_DIR}/src"
    )

    target_link_libraries(RobotNavigationTemporalTests PRIVATE
        "${RAYLIB_DIR}/build-msvc/raylib/Release/raylib.lib"
        opengl32
        gdi32
        winmm
    )

    add_test(
        NAME RobotNavigationTemporalTests
        COMMAND RobotNavigationTemporalTests
    )

    add_executable(
        RobotNavigationMotionTests
        tests/RobotMotionTests.cpp
        src/input/ManualControlleur.cpp
        ${NAVIGATION_TEST_SUPPORT_SOURCES}
    )

    target_include_directories(RobotNavigationMotionTests PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${RAYLIB_DIR}/src"
    )

    target_link_libraries(RobotNavigationMotionTests PRIVATE
        "${RAYLIB_DIR}/build-msvc/raylib/Release/raylib.lib"
        opengl32
        gdi32
        winmm
    )

    add_test(
        NAME RobotNavigationMotionTests
        COMMAND RobotNavigationMotionTests
    )

    add_executable(
        RobotNavigationEvilAhrTests
        tests/EvilAhrEventTests.cpp
        ${NAVIGATION_TEST_SUPPORT_SOURCES}
    )

    target_include_directories(RobotNavigationEvilAhrTests PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${RAYLIB_DIR}/src"
    )

    target_link_libraries(RobotNavigationEvilAhrTests PRIVATE
        "${RAYLIB_DIR}/build-msvc/raylib/Release/raylib.lib"
        opengl32
        gdi32
        winmm
    )

    add_test(
        NAME RobotNavigationEvilAhrTests
        COMMAND RobotNavigationEvilAhrTests
    )
endif()
