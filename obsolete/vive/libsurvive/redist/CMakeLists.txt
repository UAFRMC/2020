add_library(sba ./sba/sba_levmar_wrap.c
        ./sba/sba_chkjac.c
        ./sba/sba_lapack.c
        ./sba/sba_levmar.c
        ./sba/sba_crsm.c
        )

		
set(CNGFX_SRCS ./CNFG3D.c ./CNFGFunctions.c)
IF(UNIX)		
	list(APPEND CNGFX_SRCS ./CNFGXDriver.c)
	set(CNGFX_LIBS X11)
else()
	list(APPEND CNGFX_SRCS ./CNFGWinDriver.c)
endif()

add_library(CNGFX ${CNGFX_SRCS})

target_link_libraries(CNGFX ${CNGFX_LIBS})

SET(MINIMAL_OPENCV_SRCS ./minimal_opencv.c ./minimal_opencv.h)

IF(WIN32)
	set(packages_config "<packages><package id=\"lapacke\" version=\"0.1.0\" targetFramework=\"native\" /><package id=\"OpenBLAS\" version=\"0.2.14.1\" targetFramework=\"native\" /></packages>")
	file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/packages.config ${packages_config})
	ADD_DEFINITIONS(-DHAVE_LAPACK_CONFIG_H -DLAPACK_COMPLEX_STRUCTURE)
	set(MINIMAL_OPENCV_SRCS ${MINIMAL_OPENCV_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/packages.config)
endif()

add_library(minimal_opencv ${MINIMAL_OPENCV_SRCS})
IF(UNIX)
	target_link_libraries(minimal_opencv cblas lapacke m)
else()
	target_link_libraries(minimal_opencv ${CMAKE_BINARY_DIR}/packages/OpenBLAS.0.2.14.1/lib/native/lib/x64/libopenblas.dll.a)
ENDIF()

IF(WIN32)
	find_program(NUGET nuget)
	if(NOT NUGET)
		file(DOWNLOAD https://dist.nuget.org/win-x86-commandline/v3.5.0/nuget.exe "${CMAKE_CURRENT_BINARY_DIR}/nuget.exe" 
			EXPECTED_MD5 406324e1744923a530a3f45b8e4fe1eb  STATUS status LOG log)
		find_program(NUGET nuget HINTS "${CMAKE_CURRENT_BINARY_DIR}" )
	endif()
	
	add_custom_target(nuget-restore
	  COMMAND ${NUGET} restore ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.sln
	)
	set_target_properties( minimal_opencv PROPERTIES VS_USER_PROPS "$(SolutionDir)\\packages\\OpenBLAS.0.2.14.1\\build\\native\\openblas.targets" )
	set_target_properties( survive PROPERTIES VS_USER_PROPS "$(SolutionDir)\\packages\\OpenBLAS.0.2.14.1\\build\\native\\openblas.targets" )
	add_dependencies(minimal_opencv nuget-restore)
endif()

add_executable(lintest linmath.c linmath.h lintest.c)
target_link_libraries(lintest minimal_opencv)
add_test(NAME lintest COMMAND lintest)
