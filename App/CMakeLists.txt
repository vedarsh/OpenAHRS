add_library(App STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/lis2mdl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/icm45686.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ahrs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tlm.c

)

target_include_directories(App PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

target_link_libraries(App 
    stm32cubemx
    m
    )
