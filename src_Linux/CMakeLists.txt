set(CMAKE_CXX_STANDARD 20)
cmake_minimum_required(VERSION 3.16)
project(UCFO)

# Habilita el uso de información de depuración
set(CMAKE_BUILD_TYPE Debug)

# Incluye las opciones para compilación con Address Sanitizer
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -g -O0")

#Directorio que contiene las bibliotecas
set(ORBITER_ROOT "$ENV{HOME}/orbiter_test/Orbiter")
set(ORBITER_INCLUDES "${ORBITER_ROOT}/Orbitersdk/include")
set(ORBITER_LIBRARIES "${ORBITER_ROOT}/Orbitersdk/lib")

file(GLOB LIBRARIES "${ORBITER_LIBRARIES}/*.a")

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

# Opciones de compilación
add_compile_options(-fPICg)