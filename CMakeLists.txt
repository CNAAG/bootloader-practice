cmake_minimum_required(VERSION 3.28)
project(BootLoader C ASM)

# 1. 编译选项
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(CMAKE_C_FLAGS "${CPU_FLAGS} \
                    -Wall -Wextra -Werror \
                    -O2 \
                    -ffunction-sections -fdata-sections \
                    -fno-common \
                    -fno-exceptions \
                    -g3")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs")

# 2. 源文件
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/code/src/*.c)
file(GLOB_RECURSE HAL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/*.c)
list(REMOVE_ITEM HAL_SRCS 
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_timebase_tim_template.c
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_msp_template.c
)
set(STARTUP_ASM ${CMAKE_CURRENT_SOURCE_DIR}/code/startup/startup_stm32g4xx.s)

set(SOURCES ${SRCS} ${HAL_SRCS} ${STARTUP_ASM})

# 3. 创建可执行目标
add_executable(${PROJECT_NAME} ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# 4. 链接脚本
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/code/ldscript/stm32g474xx_flash.ld)
target_link_options(${PROJECT_NAME} PRIVATE
    "-T${LINKER_SCRIPT}"
    "LINKER:-Map=${PROJECT_NAME}.map"
)

# 5. 头文件目录
file(GLOB_RECURSE INCS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/STM32Cube_FW_G4_V1.6.1/Drivers/*.h)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/code/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/STM32Cube_FW_G4_V1.6.1/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/STM32Cube_FW_G4_V1.6.1/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Legacy
)

# 7. 编译宏定义
target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32G474xx
)

# 8. 生成 hex 和 bin 文件
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.bin
    COMMENT "Generating ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)

# 9. 打印固件大小
if(CMAKE_SIZE)
    add_custom_target(size ALL
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
        DEPENDS ${PROJECT_NAME}
        COMMENT "Running size on ${PROJECT_NAME}.elf"
    )
else()
    message(WARNING "CMAKE_SIZE not found. 'size' target disabled.")
endif()