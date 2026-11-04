
include(${CMAKE_CURRENT_LIST_DIR}/OpenCRGCompilerSettings.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/OpenCRGCommonIncludes.cmake)

# Convenience function to avoid code duplication when building demo/test executables
function(opencrg_build_demo_executable target)

    if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
        if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
            set(CMAKE_BUILD_TYPE Release CACHE STRING "Build Type")
            set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
        endif()
    endif()

    add_executable(${target} "src/main.c")

    set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "$<1:${CMAKE_CURRENT_SOURCE_DIR}/../bin>")

    target_link_directories(${target}
        PRIVATE ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../baselib/lib
    )

    target_link_libraries(${target}
        PRIVATE OpenCRGCompilerSettings OpenCRGCommonIncludes OpenCRG
    )

endfunction()