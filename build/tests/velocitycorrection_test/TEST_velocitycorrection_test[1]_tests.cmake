add_test( VelocityCorrection.ImuPropVelCorr /home/neofelis/VRX/drift/build/tests/velocitycorrection_test/TEST_velocitycorrection_test [==[--gtest_filter=VelocityCorrection.ImuPropVelCorr]==] --gtest_also_run_disabled_tests)
set_tests_properties( VelocityCorrection.ImuPropVelCorr PROPERTIES WORKING_DIRECTORY /home/neofelis/VRX/drift/build/tests/velocitycorrection_test)
set( TEST_velocitycorrection_test_TESTS VelocityCorrection.ImuPropVelCorr)
