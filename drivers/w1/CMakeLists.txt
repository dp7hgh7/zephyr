# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_sources_ifdef(CONFIG_USERSPACE w1_handlers.c)

# drivers implementing link functions (read, write, reset)
zephyr_library_sources_ifdef(CONFIG_W1_TEST           w1_test.c)
zephyr_library_sources_ifdef(CONFIG_W1_ZEPHYR_SERIAL  w1_zephyr_serial.c)

IF(CONFIG_W1_DS2482_800 OR CONFIG_W1_DS2484)
  zephyr_library_sources(w1_ds248x_common.c)
ENDIF()

zephyr_library_sources_ifdef(CONFIG_W1_DS2482_800 w1_ds2482-800.c)
zephyr_library_sources_ifdef(CONFIG_W1_DS2484 w1_ds2484.c)

# network functions:
if(CONFIG_W1_NET)
# network functions should be callable from user as well as supervisor mode:
remove_definitions(-D__ZEPHYR_SUPERVISOR__)
zephyr_library_sources(w1_net.c)
endif() #CONFIG_W1_NET
