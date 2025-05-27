cmake_minimum_required(VERSION 3.15)
project(PhysXWrapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

set(SOURCES
        src/pch.cpp
        src/dllmain.cpp
        src/wrapper.cpp
        src/TriggerFilterCallback.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME}
        PRIVATE
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/third_party/PhysX/include"
)

if (WIN32)
    set(PX_BIN_DIR  "${CMAKE_SOURCE_DIR}/third_party/PhysX/win32/bin")
    set(PX_LIB_DIR  "${CMAKE_SOURCE_DIR}/third_party/PhysX/win32/lib")

    target_link_directories(${PROJECT_NAME}
            PRIVATE "${PX_LIB_DIR}"
    )

    target_link_libraries(${PROJECT_NAME}
            PRIVATE
            PhysXExtensions_static_64
            PhysX_64
            PhysXCommon_64
            PhysXCooking_64
            PhysXFoundation_64
    )

    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PX_BIN_DIR}/PhysX_64.dll"        "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PX_BIN_DIR}/PhysXCommon_64.dll"  "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PX_BIN_DIR}/PhysXCooking_64.dll" "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PX_BIN_DIR}/PhysXFoundation_64.dll" "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    )

    if (MSVC)
        set_property(TARGET ${PROJECT_NAME} PROPERTY
                MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        )
    endif()

elseif (UNIX)
    set(PX_LIB_DIR  "${CMAKE_SOURCE_DIR}/third_party/PhysX/linux/lib")
    set(PX_BIN_DIR  "${CMAKE_SOURCE_DIR}/third_party/PhysX/linux/bin")

    target_link_directories(${PROJECT_NAME}
            PRIVATE
            "${PX_LIB_DIR}"
            "${PX_BIN_DIR}"
    )

    target_link_libraries(${PROJECT_NAME}
            PRIVATE
            PhysXExtensions_static_64    # .a
            PhysX_64                     # .so
            PhysXCommon_64               # .so
            PhysXCooking_64              # .so
            PhysXFoundation_64           # .so
    )

    set_target_properties(${PROJECT_NAME} PROPERTIES
            BUILD_WITH_INSTALL_RPATH TRUE
            INSTALL_RPATH "\$ORIGIN"
            INSTALL_RPATH_USE_LINK_PATH TRUE
    )
endif()
