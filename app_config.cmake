# Optional build hooks for apps under 4DNautilus/ (included by allolib_playground CMake).
# Spout is Windows-only; EoYSspout links Spout2 from 4DNautilus/Spout2/.

if(WIN32 AND this_app_name STREQUAL "EoYSspout" AND NOT TARGET Spout_static)
	set(SPOUT_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
	set(SPOUT_BUILD_SPOUTDX OFF CACHE BOOL "" FORCE)
	set(SPOUT_BUILD_CMT OFF CACHE BOOL "" FORCE)
	set(SKIP_INSTALL_ALL ON CACHE BOOL "" FORCE)
	add_subdirectory(
		"${this_app_path}/Spout2"
		"${CMAKE_BINARY_DIR}/spout2"
	)
endif()

if(WIN32 AND this_app_name STREQUAL "EoYSspout")
	list(APPEND app_include_dirs "${this_app_path}/Spout2/SPOUTSDK/SpoutGL")
	list(APPEND app_link_libs Spout_static)
endif()
