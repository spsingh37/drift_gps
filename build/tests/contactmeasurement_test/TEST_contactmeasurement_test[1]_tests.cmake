add_test( ContactMeasurementTest.Ctor /home/neofelis/VRX/drift/build/tests/contactmeasurement_test/TEST_contactmeasurement_test [==[--gtest_filter=ContactMeasurementTest.Ctor]==] --gtest_also_run_disabled_tests)
set_tests_properties( ContactMeasurementTest.Ctor PROPERTIES WORKING_DIRECTORY /home/neofelis/VRX/drift/build/tests/contactmeasurement_test)
add_test( ContactMeasurementTest.ContactSetGetBasic /home/neofelis/VRX/drift/build/tests/contactmeasurement_test/TEST_contactmeasurement_test [==[--gtest_filter=ContactMeasurementTest.ContactSetGetBasic]==] --gtest_also_run_disabled_tests)
set_tests_properties( ContactMeasurementTest.ContactSetGetBasic PROPERTIES WORKING_DIRECTORY /home/neofelis/VRX/drift/build/tests/contactmeasurement_test)
set( TEST_contactmeasurement_test_TESTS ContactMeasurementTest.Ctor ContactMeasurementTest.ContactSetGetBasic)