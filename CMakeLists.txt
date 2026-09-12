cmake_minimum_required(VERSION 3.16)

project(navion_6dof_oop_stage5_rpm_ramp_30s VERSION 5.3.0 LANGUAGES CXX)

option(NAVION_BUILD_TESTS "Build the Navion unit and integration tests" ON)
option(NAVION_BUILD_EXAMPLES "Build the Navion examples" ON)

add_library(navion6dof
    src/config/ProvisionalNavionConfig.cpp
    src/LoadAccumulator.cpp
    src/NavionModel.cpp
    src/RigidBody6DOF.cpp
    src/RK4Integrator.cpp
    src/initialization/GroundStaticTrim.cpp
    src/components/stabilizers/VATC_HorizontalStabilizer.cpp
    src/components/stabilizers/VATC_VerticalStabilizer.cpp
    src/components/fuselage/FuselageComponent.cpp
    src/components/fuselage/FuselageAerodynamics.cpp
    src/components/landing_gear/LandingGearComponent.cpp
    src/components/wings/VATC_MainWing.cpp
    src/components/propeller/PropellerModel.cpp
)

add_library(navion6dof_stage1 ALIAS navion6dof)
add_library(navion6dof_stage2 ALIAS navion6dof)
add_library(navion6dof_stage3 ALIAS navion6dof)
add_library(navion6dof_stage4 ALIAS navion6dof)
add_library(navion6dof_stage5 ALIAS navion6dof)

target_include_directories(navion6dof
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(navion6dof PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(navion6dof PRIVATE /W4 /permissive-)
else()
    target_compile_options(navion6dof PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow
    )
endif()

if(NAVION_BUILD_EXAMPLES)
    add_executable(navion_core_example
        examples/minimal_simulation.cpp
    )
    target_link_libraries(navion_core_example PRIVATE navion6dof)

    add_executable(navion_stabilizer_example
        examples/stabilizer_integration.cpp
    )
    target_link_libraries(navion_stabilizer_example PRIVATE navion6dof)

    add_executable(navion_stage3_example
        examples/stage3_integration.cpp
    )
    target_link_libraries(navion_stage3_example PRIVATE navion6dof)

    add_executable(navion_stage4_takeoff_roll
        examples/stage4_takeoff_roll.cpp
    )
    target_link_libraries(navion_stage4_takeoff_roll PRIVATE navion6dof)

    add_executable(navion_stage5_full_aircraft_takeoff_roll
        examples/stage5_full_aircraft_takeoff_roll.cpp
    )
    target_link_libraries(navion_stage5_full_aircraft_takeoff_roll PRIVATE navion6dof)

    add_executable(navion_fuselage_reference_demo
        examples/fuselage_reference_demo.cpp
    )
    target_link_libraries(navion_fuselage_reference_demo PRIVATE navion6dof)
endif()

if(NAVION_BUILD_TESTS)
    enable_testing()

    add_executable(navion_core_tests
        tests/test_stage1.cpp
    )
    target_link_libraries(navion_core_tests PRIVATE navion6dof)

    add_executable(navion_stabilizer_tests
        tests/test_stabilizers.cpp
    )
    target_link_libraries(navion_stabilizer_tests PRIVATE navion6dof)

    add_executable(navion_propeller_tests
        tests/test_propeller.cpp
    )
    target_link_libraries(navion_propeller_tests PRIVATE navion6dof)

    add_executable(navion_landing_gear_tests
        tests/test_landing_gear.cpp
    )
    target_link_libraries(navion_landing_gear_tests PRIVATE navion6dof)

    add_executable(navion_main_wing_tests
        tests/test_main_wing.cpp
    )
    target_link_libraries(navion_main_wing_tests PRIVATE navion6dof)

    add_executable(navion_fuselage_tests
        tests/test_fuselage.cpp
    )
    target_link_libraries(navion_fuselage_tests PRIVATE navion6dof)

    add_executable(navion_full_aircraft_tests
        tests/test_full_aircraft.cpp
    )
    target_link_libraries(navion_full_aircraft_tests PRIVATE navion6dof)

    add_executable(navion_ground_static_trim_tests
        tests/test_ground_static_trim.cpp
    )
    target_link_libraries(navion_ground_static_trim_tests PRIVATE navion6dof)

    if(MSVC)
        target_compile_options(navion_core_tests PRIVATE /W4 /permissive-)
        target_compile_options(navion_stabilizer_tests PRIVATE /W4 /permissive-)
        target_compile_options(navion_propeller_tests PRIVATE /W4 /permissive-)
        target_compile_options(navion_landing_gear_tests PRIVATE /W4 /permissive-)
        target_compile_options(navion_main_wing_tests PRIVATE /W4 /permissive-)
        target_compile_options(navion_fuselage_tests PRIVATE /W4 /permissive-)
        target_compile_options(navion_full_aircraft_tests PRIVATE /W4 /permissive-)
        target_compile_options(navion_ground_static_trim_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(navion_core_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(navion_stabilizer_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(navion_propeller_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(navion_landing_gear_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(navion_main_wing_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(navion_fuselage_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(navion_full_aircraft_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(navion_ground_static_trim_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
    endif()

    add_test(NAME navion_core_tests COMMAND navion_core_tests)
    add_test(NAME navion_stabilizer_tests COMMAND navion_stabilizer_tests)
    add_test(NAME navion_propeller_tests COMMAND navion_propeller_tests)
    add_test(NAME navion_landing_gear_tests COMMAND navion_landing_gear_tests)
    add_test(NAME navion_main_wing_tests COMMAND navion_main_wing_tests)
    add_test(NAME navion_fuselage_tests COMMAND navion_fuselage_tests)
    add_test(NAME navion_full_aircraft_tests COMMAND navion_full_aircraft_tests)
    add_test(NAME navion_ground_static_trim_tests COMMAND navion_ground_static_trim_tests)
endif()
