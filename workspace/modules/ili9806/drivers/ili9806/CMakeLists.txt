# Declares the current directory as a Zephyr library
# If no name is given, the name is derived from the directory name
zephyr_library()

# List the source code files for the library
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
zephyr_library_sources_ifdef(CONFIG_ILI9806 display_ili9806.c)