# Automatically generated from BUILD in util/thread

add_library(icon_shared_memory_util_thread_lockstep STATIC
  "${CMAKE_CURRENT_LIST_DIR}/lockstep.cc"
  "${CMAKE_CURRENT_LIST_DIR}/lockstep.h"
)
target_include_directories(icon_shared_memory_util_thread_lockstep PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_util_thread_lockstep PUBLIC
  icon_shared_memory_icon_interprocess_binary_futex
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_time
)
install(TARGETS icon_shared_memory_util_thread_lockstep
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/lockstep.h"
        DESTINATION "include/util/thread"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_util_thread_lockstep_test
    "${CMAKE_CURRENT_LIST_DIR}/lockstep_test.cc"
  )
  target_include_directories(icon_shared_memory_util_thread_lockstep_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_util_thread_lockstep_test PRIVATE
    icon_shared_memory_util_thread_lockstep
    icon_shared_memory_icon_utils_log
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    icon_shared_memory_icon_utils_time
    GTest::gtest
    GTest::gtest_main
  )
  gtest_add_tests(TARGET icon_shared_memory_util_thread_lockstep_test)
endif()
