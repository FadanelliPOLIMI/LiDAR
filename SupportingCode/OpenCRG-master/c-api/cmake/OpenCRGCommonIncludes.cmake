if(NOT TARGET OpenCRGCommonIncludes)
    add_library(OpenCRGCommonIncludes INTERFACE)

    # Include directories
    target_include_directories(OpenCRGCommonIncludes INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/../baselib/inc
    )
endif()