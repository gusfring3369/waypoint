add_executable(waypoint_tests
    main.cpp
    test_geo.cpp
    test_quad_tree.cpp
    test_road_network.cpp
    test_shortest_path_engine.cpp
    test_matching.cpp
    test_dispatcher.cpp)
target_link_libraries(waypoint_tests PRIVATE waypoint::waypoint)
target_include_directories(waypoint_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME waypoint_tests COMMAND waypoint_tests)
