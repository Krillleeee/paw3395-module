# Copyright (c) 2022 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PAW3395 src/paw3395.c)
zephyr_library_sources_ifdef(CONFIG_PAW3395 src/paw3395_priv.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
