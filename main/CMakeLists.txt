idf_component_register(
    SRCS "M5CoreAnalyser.c"
    INCLUDE_DIRS "."
    REQUIRES bsp_m5core2 recorder_sink tool_core ui_shell
)
