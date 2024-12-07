# Demo Tests

These are tests that do not form part of the unit test suite, and are meant for demonstation, experimentation and prototyping of design patters or the like.

To add a demo test:
1. Create a directory with the name of your test
2. Add the directory with `add_subdirectory` to the `demo_tests/CMakeLists.txt`.
3. Create a `CMakeLists.txt` file in the test directory and add the executable with `add_executable(exec_name source_file.cpp)`