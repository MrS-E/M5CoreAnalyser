idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    REQUIRES unity tool_core bsp_m5core2 recorder_sink
)

target_include_directories(${COMPONENT_LIB} PRIVATE
    "../priv_include"
    "../../bsp_m5core2/priv_include"
)
