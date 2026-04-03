add_executable(example_basic_spprc basic_spprc.cpp)
target_link_libraries(example_basic_spprc PRIVATE bgspprc)
target_compile_options(example_basic_spprc PRIVATE -Wall -Wextra -Wpedantic -O2)

add_executable(example_custom_resource custom_resource.cpp)
target_link_libraries(example_custom_resource PRIVATE bgspprc)
target_compile_options(example_custom_resource PRIVATE -Wall -Wextra -Wpedantic -O2)
