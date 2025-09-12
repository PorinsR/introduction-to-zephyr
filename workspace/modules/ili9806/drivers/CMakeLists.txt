# Custom Zephyr function that imports the ili9806/ subdirectory if the Kconfig
# option ILI9806 is defined
add_subdirectory_ifdef(CONFIG_ILI9806 ili9806)