if(EXISTS "/home/neofelis/VRX/drift/build/tests/test_example_1/TEST_test_example_1[1]_tests.cmake")
  include("/home/neofelis/VRX/drift/build/tests/test_example_1/TEST_test_example_1[1]_tests.cmake")
else()
  add_test(TEST_test_example_1_NOT_BUILT TEST_test_example_1_NOT_BUILT)
endif()