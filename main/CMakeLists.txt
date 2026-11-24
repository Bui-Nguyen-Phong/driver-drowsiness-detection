set(srcs "main.c"
         "web_cam_server.c"
         "app_camera.c"
         "face_pipeline.cpp"
         "audio_alert.c")
set(html_files "../frontend/gzipped/index.html.gz"
               "../frontend/gzipped/loading.jpg.gz"
               "../frontend/gzipped/favicon.ico.gz"
               "../frontend/gzipped/assets/index.js.gz"
               "../frontend/gzipped/assets/index.css.gz")

idf_component_register(SRCS "${srcs}"
                       PRIV_INCLUDE_DIRS .
                       EMBED_TXTFILES ${html_files})

# PFLD landmark model, embedded aligned in flash rodata (symbol: _binary_pfld_espdl_start)
idf_build_get_property(component_targets __COMPONENT_TARGETS)
if("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif("___idf_esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()
set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
include(${cmake_dir}/utilities.cmake)
target_add_aligned_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_SOURCE_DIR}/../models/pfld.espdl" BINARY)
# Eye open/closed classifier (open-closed-eye-0001, INT8), symbol: _binary_open_closed_eye_espdl_start
target_add_aligned_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_SOURCE_DIR}/../models/open_closed_eye.espdl" BINARY)
