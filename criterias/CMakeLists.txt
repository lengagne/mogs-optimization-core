
# FIXME When smart dealing of constraints and criterias.

# file(GLOB all_valid_subdirs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*/CMakeLists.txt")
# 
# foreach(dir ${all_valid_subdirs})
#     if(${dir} MATCHES "^([^/]*)/CMakeLists.txt")
#         string(REGEX REPLACE "^([^/]*)/CMakeLists.txt" "\\1" dir_trimmed ${dir})
#         add_subdirectory(${dir_trimmed})
#      endif()
# endforeach(dir)
