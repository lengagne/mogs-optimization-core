# add all the subdirectories

#file(GLOB all_valid_subdirs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*/CMakeLists.txt")

add_subdirectory(BoxCollisionConstraint)
add_subdirectory(BoxContactConstraint)
add_subdirectory(BalanceConstraint)
add_subdirectory(PositionConstraint)

#foreach(dir ${all_valid_subdirs})
#    message(STATUS "path = ${dir}")
#    if(${dir} MATCHES "^([^/]*)//CMakeLists.txt")
#        string(REGEX REPLACE "^([^/]*)//CMakeLists.txt" "\\1" dir_trimmed ${dir})	
#        add_subdirectory(${dir_trimmed})
#    endif()
#endforeach(dir)

