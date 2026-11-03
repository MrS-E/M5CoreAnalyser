cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "../components")
set(TEST_COMPONENTS "bsp_m5core2")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(m5core_analyser_test_app)
