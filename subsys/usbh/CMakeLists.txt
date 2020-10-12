# Copyright (c) 2020 PHYTEC Messtechnik GmbH
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_USBH)
	zephyr_library()
	zephyr_library_sources_ifdef(CONFIG_UC_USBH_MSC class/usbh_msc.c)

	zephyr_include_directories(${ZEPHYR_BASE}/subsys/usbh class)

	zephyr_library_sources(
		usbh_class.c
		usbh_core.c
		usbh_hub.c
		)
	endif()
