# Automatically generated from BUILD in icon/testing

add_library(icon_shared_memory_icon_testing_realtime_annotations INTERFACE)
target_sources(icon_shared_memory_icon_testing_realtime_annotations PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/realtime_annotations.h"
)
target_include_directories(icon_shared_memory_icon_testing_realtime_annotations INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_testing_realtime_annotations
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/realtime_annotations.h"
        DESTINATION "include/icon/testing"
)
