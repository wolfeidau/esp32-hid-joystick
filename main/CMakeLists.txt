set(COMPONENT_SRCS "joystick_main.c"
    "esp_hidd_prf_api.c"
    "hid_dev.c"
    "hid_device_le_prf.c"
    "joystick_buttons.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

if(GCC_NOT_5_2_0)
    component_compile_options(-Wno-unused-const-variable)
    endif()
