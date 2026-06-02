# Spout static link for EoYSspout (Windows only). Included from flags.cmake / app_config.cmake.
# CMAKE_CURRENT_LIST_DIR is 4DNautilus/ when this file is included.

if(CMAKE_SYSTEM_NAME MATCHES "Windows"
	AND DEFINED this_app_name
	AND this_app_name STREQUAL "EoYSspout")

	set(_4D_NAUTILUS_DIR "${CMAKE_CURRENT_LIST_DIR}")

	if(NOT EXISTS "${_4D_NAUTILUS_DIR}/Spout2/SPOUTSDK/SpoutGL/SpoutSender.h")
		message(FATAL_ERROR
			"[4DNautilus] Spout2 missing. Clone:\n"
			"  git clone https://github.com/leadedge/Spout2.git \"${_4D_NAUTILUS_DIR}/Spout2\"")
	endif()

	if(NOT TARGET Spout_static)
		message(STATUS "[4DNautilus] SpoutBuild: add_subdirectory Spout2 -> Spout_static")
		set(SPOUT_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
		set(SPOUT_BUILD_SPOUTDX OFF CACHE BOOL "" FORCE)
		set(SPOUT_BUILD_CMT OFF CACHE BOOL "" FORCE)
		set(SKIP_INSTALL_ALL ON CACHE BOOL "" FORCE)
		add_subdirectory(
			"${_4D_NAUTILUS_DIR}/Spout2"
			"${CMAKE_BINARY_DIR}/spout2"
		)
	endif()

	list(APPEND app_include_dirs "Spout2/SPOUTSDK/SpoutGL")
	list(APPEND app_link_libs Spout_static)

	message(STATUS "[4DNautilus] SpoutBuild: OK (include + link Spout_static)")

else()
	if(NOT CMAKE_SYSTEM_NAME MATCHES "Windows")
		message(STATUS "[4DNautilus] SpoutBuild: skipped (CMAKE_SYSTEM_NAME=${CMAKE_SYSTEM_NAME})")
	else()
		message(STATUS "[4DNautilus] SpoutBuild: skipped (this_app_name='${this_app_name}')")
	endif()
endif()
