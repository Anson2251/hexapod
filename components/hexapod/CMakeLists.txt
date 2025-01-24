idf_component_register(
    SRCS "hexapod.c"
    INCLUDE_DIRS "include"
    REQUIRES my_servo driver log
)
