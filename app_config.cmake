# Included by allolib_playground CMake when building apps in this folder.
# Spout: static link from 4DNautilus/Spout2/ (no separate DLL build required).

if(WIN32 AND this_app_name STREQUAL "EoYSspout")
	# Directory containing this file (4DNautilus/)
	set(_4D_NAUTILUS_DIR "${CMAKE_CURRENT_LIST_DIR}")

	if(NOT EXISTS "${_4D_NAUTILUS_DIR}/Spout2/SPOUTSDK/SpoutGL/SpoutSender.h")
		message(FATAL_ERROR
			"Spout2 not found. Clone into 4DNautilus/Spout2:\n"
			"  git clone https://github.com/leadedge/Spout2.git ${_4D_NAUTILUS_DIR}/Spout2")
	endif()

	if(NOT TARGET Spout_static)
		set(SPOUT_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
		set(SPOUT_BUILD_SPOUTDX OFF CACHE BOOL "" FORCE)
		set(SPOUT_BUILD_CMT OFF CACHE BOOL "" FORCE)
		set(SKIP_INSTALL_ALL ON CACHE BOOL "" FORCE)
		add_subdirectory(
			"${_4D_NAUTILUS_DIR}/Spout2"
			"${CMAKE_BINARY_DIR}/spout2"
		)
	endif()

	list(APPEND app_include_dirs "${_4D_NAUTILUS_DIR}/Spout2/SPOUTSDK/SpoutGL")
	list(APPEND app_link_libs Spout_static)
endif()
