# Copyright (c) 2022 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_include_directories(.)
zephyr_include_directories_ifdef(CONFIG_PMW3320)

zephyr_library()

zephyr_library_sources(pmw3320.c)
zephyr_library_sources_ifdef(CONFIG_PMW3320_TRIGGER pmw3320_trigger.c)
