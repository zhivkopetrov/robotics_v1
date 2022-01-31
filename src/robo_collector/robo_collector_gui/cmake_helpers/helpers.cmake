cmake_minimum_required(VERSION 3.10.2)

list(APPEND 
    CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/find_modules)

function(set_target_cpp_standard target standard)
set_target_properties(
    ${target}
    PROPERTIES
        CXX_STANDARD ${standard}
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)
endfunction()

function(enable_target_warnings target)
    target_compile_options(
        ${target}
            PRIVATE
              -Wall
              -Wextra
              -Werror
              -Wundef
              -Wuninitialized
              -Wshadow
              -Wpointer-arith
              -Wcast-align
              -Wcast-qual
              -Wunused-parameter
              -Wdouble-promotion
              -Wnull-dereference
    )
    
    if(CMAKE_CXX_COMPILER_ID MATCHES GNU AND NOT ${USE_IWYU})
        #supported only in GNU
        #however include-what-you-use is not happy with those options
        target_compile_options(
          ${target}
              PRIVATE
                -Wlogical-op
                -Wduplicated-cond
                -Wduplicated-branches
        )
    endif()
endfunction()

function(enable_target_include_what_you_use target)
    find_package(IWYU REQUIRED)
    set_target_properties(
    ${target}
        PROPERTIES 
        CXX_INCLUDE_WHAT_YOU_USE ${IWYU_BINARY_PATH}
    )
endfunction()

function(enable_target_position_independent_code target)
    set_target_properties(
        ${target}
            PROPERTIES 
            POSITION_INDEPENDENT_CODE ON
    )
endfunction()

function(set_target_visibility target)
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(DEFAULT_BUILD_TYPE "Debug")
            message(STATUS 
              "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
            set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
                STRING "Choose the type of build." FORCE)
    endif()
    
    if(${CMAKE_BUILD_TYPE} MATCHES Release OR 
       ${CMAKE_BUILD_TYPE} MATCHES MinSizeRel)
        # Default to hidden visibility for symbols
        set(CMAKE_CXX_VISIBILITY_PRESET hidden)
        set(CMAKE_VISIBILITY_INLINES_HIDDEN TRUE)
        
        set_target_properties(
            ${target}
             PROPERTIES 
                CXX_VISIBILITY_PRESET hidden
                VISIBILITY_INLINES_HIDDEN TRUE
        )
    elseif(${CMAKE_BUILD_TYPE} MATCHES Debug OR 
           ${CMAKE_BUILD_TYPE} MATCHES RelWithDebInfo)  
        if(UNIX)
            set(R_DYNAMIC_FLAG "-rdynamic")
        elseif(APPLE)
            set(R_DYNAMIC_FLAG "-Wl,-export_dynamic")
        endif()
    endif()
    
    target_link_libraries(
        ${target} 
            PRIVATE
            ${R_DYNAMIC_FLAG} # export of static symbols
    )   
endfunction()

#enable_target_c_sanitizer(${my_target} "address")
# Available sanitizers
#
# GCC: address, thread, leak, undefined
# CLANG: address, memory, thread, leak, undefined
function(enable_target_sanitizer target sanitizer)
    if(NOT CMAKE_BUILD_TYPE OR NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
        message(
            FATAL_ERROR
            "Error: Sanitizers can be enabled only with 'Debug' build\n"
            "Hint: Use 'cmake .. -DCMAKE_BUILD_TYPE=Debug'")
        return()
    endif()
    
    target_link_libraries(
        ${target}
                -fsanitize=${sanitizer}
                -fsanitize-recover=all
    )
    
    target_compile_options(
        ${target}
                -fsanitize-recover=all
    )
    
    if(${sanitizer} STREQUAL "address")
        target_link_libraries(
            ${target}
                    -fno-omit-frame-pointer
        )
    endif()
    
    if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
        if (${sanitizer} STREQUAL "undefined")
            target_link_libraries(
                ${target}
                        -lubsan
            )
        elseif (${sanitizer} STREQUAL "thread")
            target_link_libraries(
                ${target}
                        -ltsan
            )
        endif()
    endif()
endfunction()

function(install_and_export_target target include_folder_name)
    install(
        TARGETS ${target} EXPORT ${target}_targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    
    install(
        DIRECTORY ${include_folder_name}/
        DESTINATION ${include_folder_name}
    )
    
    install(
        EXPORT ${target}_targets
        DESTINATION lib/cmake/${target}
        FILE ${target}Targets.cmake
        NAMESPACE ${target}::
    )
    
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${target}ConfigVersion.cmake 
        VERSION 1.0.0
        COMPATIBILITY SameMajorVersion
    )
    
    install(
        FILES ${target}Config.cmake
        DESTINATION lib/cmake/${target}
    )
endfunction()

# Requires package.xml file to be present in the current directory
function(enable_ros_tooling_for_target target package_xml)
    find_package(ament_cmake REQUIRED)

    # Install package.xml file so this package can be processed by ROS toolings
    # Installing this in non-ROS environments won't have any effect, but it won't harm, either.
    install(
        FILES ${package_xml} 
        DESTINATION share/${target}
    )
    
    # ros2 run requires both libraries and binaries to be placed in 'lib'
    install(TARGETS
      ${target}
      DESTINATION lib/${target}
    )
    
    # Allows Colcon to find non-Ament packages when using workspace underlays
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${target} "")
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${target} DESTINATION share/ament_index/resource_index/packages)
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${target}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${target}/hook/ament_prefix_path.dsv DESTINATION share/${target}/hook)
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${target}/hook/ros_package_path.dsv "prepend-non-duplicate;ROS_PACKAGE_PATH;")
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${target}/hook/ros_package_path.dsv DESTINATION share/${target}/hook)

    ament_package()
endfunction()
