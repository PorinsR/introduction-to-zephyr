cmake_minimum_required(VERSION 3.20.0)

# Add additional modules (before find_package)
# set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../../drivers/i2c/tca954x")
# set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../../modules/ili9806")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(powerboard)
# add_subdirectory(drivers)
target_sources(app PRIVATE src/main.c)