# Add C++ source files instead of the original C file
set(MAIN_SOURCES 
    "main.cpp"              # Main application entry point
    "display_driver.cpp" 
    ) 

# Include all source files for LVGL demos
set(LV_DEMO_DIR ../managed_components/lvgl__lvgl/demos) # Path to LVGL demo components
file(GLOB_RECURSE LV_DEMOS_SOURCES ${LV_DEMO_DIR}/*.c)  # Recursively find all .c files in the demo directory

# Register the component with ESP-IDF build system
idf_component_register(
    SRCS ${MAIN_SOURCES} ${DEMO_UI_SOURCES} ${IMAGE_SOURCES} ${LV_DEMOS_SOURCES} # All source files
    INCLUDE_DIRS "." ${LV_DEMO_DIR} # Include the current directory and demo directory for headers
)

# Set the compiler for C++ support (optional, usually already set in ESP-IDF)
set_source_files_properties(
    ${LV_DEMOS_SOURCES} 
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE # Use simplified include paths for LVGL
)

# Configure the C++ standard to C++17
# Setting compilation flags for both C and C++ files
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-std=c99>         # Use C99 for C source files
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>     # Use C++20 for C++ source files
    #$<$<COMPILE_LANGUAGE:CXX>:-std=c++17>     # Use C++17 for C++ source files
)

#