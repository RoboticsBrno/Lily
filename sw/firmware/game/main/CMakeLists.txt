idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver pthread spiffs vfs fatfs
)

target_compile_options(${COMPONENT_LIB} PRIVATE -fconcepts-diagnostics-depth=2)
