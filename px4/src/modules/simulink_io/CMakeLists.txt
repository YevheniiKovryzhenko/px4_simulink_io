file(GLOB SIMULINK_GENERATED_SRCS "generated_code/*.cpp" "generated_code/*.c")


px4_add_module(
    MODULE modules__simulink_io
    MAIN simulink_io
    STACK_MAIN 2500
    SRCS
        simulink_io.cpp
        ${SIMULINK_GENERATED_SRCS}
    INCLUDES
        generated_code
)
