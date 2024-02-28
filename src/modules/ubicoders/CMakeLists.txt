px4_add_module(
    MODULE modules__ubicoders # [modules]__[module name]
    MAIN ubicoders_simple_app # the main function name
    SRCS
        simple_app.hpp # the .hpp file name
        simple_app.cpp # the .cpp file name

    DEPENDS
        px4_work_queue
    )
