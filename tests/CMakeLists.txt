project(densepoints_tests)

add_definitions(${DENSEPOINTS_DEFINITIONS})

# Files used by tests
add_definitions(
  -DDENSEPOINTS_TESTS_OUTPUT_PATH="${DENSEPOINTS_TESTS_OUTPUT_PATH}"
)

# Rule: Each file will contain a single test
file(GLOB_RECURSE DENSEPOINTS_TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

# Create an executable for each test
foreach(TEST_SOURCE_FILE ${DENSEPOINTS_TEST_SOURCES})
  # Get the filename part (no path)
  get_filename_component(TEST_SOURCE_FILENAME ${TEST_SOURCE_FILE} NAME)
  # Convert relative path to prefix
  get_filename_component(TEST_SOURCE_DIRECTORY ${TEST_SOURCE_FILE} DIRECTORY)
  string(REPLACE "[/\\]" "_" TEST_PREFIX ${TEST_SOURCE_DIRECTORY})
  # Remove the extension
  string(REPLACE ".cpp" "" TEST_NAME ${TEST_SOURCE_FILENAME})
  set(TEST_NAME "${TEST_PREFIX}_${TEST_NAME}")
  add_executable(${TEST_NAME} ${TEST_SOURCE_FILE})
  target_link_libraries(${TEST_NAME} ${DENSEPOINTS_LIBS} ${DENSEPOINTS_THIRD_PARTY_LIBS} ${GTEST_BOTH_LIBRARIES})
  # Add to GTest
  GTEST_ADD_TESTS(${TEST_NAME} "${TEST_ARGS}" AUTO)
endforeach()
