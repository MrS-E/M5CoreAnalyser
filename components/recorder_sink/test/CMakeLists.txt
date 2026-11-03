idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    REQUIRES unity recorder_sink
)

target_include_directories(${COMPONENT_LIB} PRIVATE "../priv_include")
