
    foreach(p LIB INCLUDE)
        set(var CMAKE_INSTALL_${p}DIR)
        if(NOT IS_ABSOLUTE "${${var}}")
            set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
        endif()
    endforeach()


    include_directories (
        include
    )

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    FILE ( GLOB_RECURSE
            header_files_MogsCriteria
            include/*.h)

    FILE ( GLOB_RECURSE
            source_files_MogsCriteria
            src/*.cpp)

    ADD_LIBRARY(	MogsCriteria SHARED
            ${source_files_MogsCriteria}
    )

#    target_link_libraries(  MogsCriteria
#                            ${MogsCriteria_LIBRARIES}
#                            ${ADOLC_LIBRARIES}
#                            ${MogsIpoptOptimization_LIBRARIES}
#                            ${MogsVisu_LIBRARIES})

	link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
						)


	SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    # install the include files
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsCriteria)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsCriteria} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsCriteria LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    SET(MogsCriteria_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
    SET(MogsCriteria_LIBRARIES  MogsCriteria CACHE INTERNAL "")
    SET(MogsCriteria_FLAGS "-DMogsCriteria_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
    add_definitions(${MogsCriteria_FLAGS} ${MogsVisu_FLAGS})
    #  add flags (done before)


#    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsCriteriaConfig.cmake"
#            DESTINATION "${INSTALL_LIB_PATH}/MogsCriteria" COMPONENT dev )
