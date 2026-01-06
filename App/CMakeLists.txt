add_library(App STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/lis2mdl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/icm45686.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ds3231.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/nav/ahrs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tlm/tlm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors/imu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors/mag.c

)

target_include_directories(App PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/nav
    ${CMAKE_CURRENT_SOURCE_DIR}/tlm
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors
)

target_link_libraries(App 
    stm32cubemx
    m
    )
