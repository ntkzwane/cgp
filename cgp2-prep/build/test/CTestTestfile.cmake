# CMake generated Testfile for 
# Source directory: /home/chuck/Downloads/Downloads/cgp2-prep/test
# Build directory: /home/chuck/Downloads/Downloads/cgp2-prep/build/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(smoketest "/home/chuck/Downloads/Downloads/cgp2-prep/build/test/tilertest" "-v" "--test=commit")
SET_TESTS_PROPERTIES(smoketest PROPERTIES  WORKING_DIRECTORY "/home/chuck/Downloads/Downloads/cgp2-prep/build")
