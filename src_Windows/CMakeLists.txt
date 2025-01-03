set(CMAKE_CXX_STANDARD 20)
cmake_minimum_required(VERSION 3.16)
project(UCFO_for_Windows)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/BIN)

# Habilita el uso de información de depuración
set(CMAKE_BUILD_TYPE Release)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if (MSVC)
    # Deshabilitar advertencias específicas
    add_compile_options(/wd4996)  # Advertencia sobre 'strnicmp' deprecado
    add_compile_options(/wd5999)  # Advertencia sobre la conversión de tipos
    add_compile_options(/WX-)     # No tratar advertencias como errores
    add_compile_options(/we4311 /DNOMINMAX)
    add_compile_options(/permissive)
endif()

#Directorio que contiene las bibliotecas
set(LIBRARY_DIRECTORY "C:\\Users\\Matías Saibene\\Saved Games\\OpenOrbiter_2024\\Orbitersdk\\lib")

file(GLOB LIBRARIES "${LIBRARY_DIRECTORY}/*.lib")

# Definiciones de los archivos de código fuente
set(SOURCE_FILES
    main.cpp
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
    "C:\\Users\\Matías Saibene\\Saved Games\\OpenOrbiter_2024\\Orbitersdk\\HEADERS"
)

# Enlazar con todas las bibliotecas encontradas en el directorio
target_link_libraries(UCFO_for_Windows ${LIBRARIES})

# Opciones de compilación
#add_compile_options(-fPICg)