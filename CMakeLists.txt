cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# NAPT forgets UDP mappings after 2 s by default; slow DNS replies then get dropped
idf_build_set_property(COMPILE_DEFINITIONS "IP_NAPT_TIMEOUT_MS_UDP=30000" APPEND)
project(palm-bridge)
