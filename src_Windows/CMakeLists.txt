set(CMAKE_CXX_STANDARD 20)
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_CXX_EXTENSIONS ON)
project(UCFO)

if (MSVC)
    # Deshabilitar advertencias específicas
    add_compile_options(/wd4996)  # Advertencia sobre 'strnicmp' deprecado
    add_compile_options(/wd2664)  # Advertencia sobre la conversión de tipos
    add_compile_options(/WX-)     # No tratar advertencias como errores
    add_compile_options(/we4311 /DNOMINMAX)
    add_compile_options(/permissive)
endif()

set(ORBITER_ROOT "C:\\Users\\Matias_Saibene\\Desktop\\OpenOrbiter2024")
set(ORBITER_SDK_DIRECTORY "${ORBITER_ROOT}/Orbitersdk")
set(LIBRARY_DIRECTORY "${ORBITER_SDK_DIRECTORY}/lib")
set(INCLUDE_DIRECTORY "${ORBITER_SDK_DIRECTORY}/include")
set(XRSOUND_DIRECTORY "${ORBITER_SDK_DIRECTORY}/XRSound")

file(GLOB LIBRARIES "${LIBRARY_DIRECTORY}/*.lib")
file(GLOB XRSOUNDLIBRARIES "${XRSOUND_DIRECTORY}/*.lib")

# Definiciones de los archivos de código fuente
set(SOURCE_FILES
    main.cpp
    get_help.cpp
    make_anim.cpp
    make_annotation.cpp
    make_light.cpp
    set_engine.cpp
    set_feature.cpp
    set_contact.cpp
    set_anim.cpp
    set_annotation.cpp
)

# Definiciones de los archivos de encabezado
set(HEADER_FILES
    main.h
)

# Agregar los archivos de código fuente y encabezado al proyecto
add_library(${PROJECT_NAME} SHARED
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

# Especificar directorios de inclusión
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ORBITER_INCLUDES}
)

# Enlazar con todas las bibliotecas encontradas en el directorio
target_link_libraries(UCFO ${LIBRARIES})

if(WIN32)
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /MD)

    # Librerías de tiempo de ejecución multi-threaded
    target_compile_options(${PROJECT_NAME} PRIVATE /MD)

    # Opciones de optimización y depuración
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        target_compile_options(${PROJECT_NAME} PRIVATE /O2)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE /Zi /Od)
        target_link_options(${PROJECT_NAME} PRIVATE /DEBUG)
    endif()
endif()
