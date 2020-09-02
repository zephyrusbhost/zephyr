if (CONFIG_USBH)
	zephyr_library()
	zephyr_library_sources_ifdef(CONFIG_UC_USBH_MSC Class/MSC/usbh_msc.c)
	zephyr_library_sources_ifdef(CONFIG_UC_USBH_CDC Class/CDC/usbh_cdc.c)
	zephyr_library_sources_ifdef(CONFIG_UC_USBH_CDC Class/CDC/ACM/usbh_acm.c)

	#samr21 board support package and host controller driver
	zephyr_library_sources_ifdef(CONFIG_BOARD_ATSAMR21_XPRO HCD/ATSAMx/usbh_hcd_atsamx.c)
	zephyr_library_sources_ifdef(CONFIG_BOARD_ATSAMR21_XPRO bsp/bsp_usbh_samr21.c)

	zephyr_include_directories(${ZEPHYR_BASE}/subsys/usbh HCD/ATSAMx Class/MSC Class/CDC Class/CDC/ACM bsp Source)

	zephyr_library_sources(
		Source/usbh_class.c
		Source/usbh_core.c
		Source/usbh_hub.c
		#       usbh_os .c
		usbh_hc_cfg.c
		)
	endif()
