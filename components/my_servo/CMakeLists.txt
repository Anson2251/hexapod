idf_component_register(
    SRCS "my_servo.c"          # 组件的源文件
    INCLUDE_DIRS "include"         # 公共头文件目录
    REQUIRES log driver            # 依赖的其他组件
)