# CMake generated Testfile for 
# Source directory: /home/jwrig/EGH400-2/src/offboard_control
# Build directory: /home/jwrig/EGH400-2/build/offboard_control
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(lint_cmake "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/jwrig/EGH400-2/build/offboard_control/test_results/offboard_control/lint_cmake.xunit.xml" "--package-name" "offboard_control" "--output-file" "/home/jwrig/EGH400-2/build/offboard_control/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/iron/bin/ament_lint_cmake" "--xunit-file" "/home/jwrig/EGH400-2/build/offboard_control/test_results/offboard_control/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/jwrig/EGH400-2/src/offboard_control" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/iron/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/iron/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/iron/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/iron/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/iron/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/iron/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/iron/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/jwrig/EGH400-2/src/offboard_control/CMakeLists.txt;26;ament_package;/home/jwrig/EGH400-2/src/offboard_control/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/jwrig/EGH400-2/build/offboard_control/test_results/offboard_control/xmllint.xunit.xml" "--package-name" "offboard_control" "--output-file" "/home/jwrig/EGH400-2/build/offboard_control/ament_xmllint/xmllint.txt" "--command" "/opt/ros/iron/bin/ament_xmllint" "--xunit-file" "/home/jwrig/EGH400-2/build/offboard_control/test_results/offboard_control/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/jwrig/EGH400-2/src/offboard_control" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/iron/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/iron/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/iron/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/iron/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/iron/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/iron/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/iron/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/jwrig/EGH400-2/src/offboard_control/CMakeLists.txt;26;ament_package;/home/jwrig/EGH400-2/src/offboard_control/CMakeLists.txt;0;")
