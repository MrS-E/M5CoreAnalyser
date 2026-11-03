cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "../components" "../managed_components")
set(TEST_COMPONENTS "bsp_m5core2" "recorder_sink" "tool_core" "ui_shell" "tool_voltmeter" "tool_scope")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(m5core_analyser_test_app)
