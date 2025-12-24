add_library(App STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lis2mdl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/icm45686.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ahrs.c

)

target_include_directories(App PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(App 
    stm32cubemx
    m
    )
