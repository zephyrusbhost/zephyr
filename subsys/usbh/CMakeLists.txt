add_subdirectory(hcd)

if (CONFIG_USBH)
	zephyr_library()
	zephyr_library_sources_ifdef(CONFIG_UC_USBH_MSC class/usbh_msc.c)
	#samr21 board support package and host controller driver
	#zephyr_library_sources_ifdef(CONFIG_BOARD_ATSAMR21_XPRO bsp/bsp_usbh_samr21.c)

	zephyr_include_directories(${ZEPHYR_BASE}/subsys/usbh hcd class)

	zephyr_library_sources(
		usbh_class.c
		usbh_core.c
		usbh_hub.c
		)
	endif()
