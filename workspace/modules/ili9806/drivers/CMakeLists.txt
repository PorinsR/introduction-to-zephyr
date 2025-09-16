# Custom Zephyr function that imports the mcp9808/ subdirectory if the Kconfig
# option ILI9806 is defined
add_subdirectory_ifdef(CONFIG_ILI9806 ili9806)