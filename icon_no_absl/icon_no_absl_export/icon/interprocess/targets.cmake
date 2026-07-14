# Automatically generated from BUILD in icon/interprocess

add_library(icon_shared_memory_icon_interprocess_binary_futex STATIC
  "${CMAKE_CURRENT_LIST_DIR}/binary_futex.cc"
  "${CMAKE_CURRENT_LIST_DIR}/binary_futex.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_binary_futex PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_binary_futex PUBLIC
  icon_shared_memory_icon_utils_attributes
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_strerror
  icon_shared_memory_icon_utils_time
)
install(TARGETS icon_shared_memory_icon_interprocess_binary_futex
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/binary_futex.h"
        DESTINATION "include/icon/interprocess"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_binary_futex_test
    "${CMAKE_CURRENT_LIST_DIR}/binary_futex_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_binary_futex_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_binary_futex_test PRIVATE
    icon_shared_memory_icon_interprocess_binary_futex
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    icon_shared_memory_icon_utils_time
    GTest::gtest
    GTest::gtest_main
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_binary_futex_test)
endif()
