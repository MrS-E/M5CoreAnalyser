idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    REQUIRES unity tool_scope ui_shell tool_core recorder_sink bsp_m5core2 lvgl
)

target_include_directories(${COMPONENT_LIB} PRIVATE
    "../priv_include"
    "../../bsp_m5core2/priv_include"
    "../../recorder_sink/priv_include"
    "../../tool_core/priv_include"
)
