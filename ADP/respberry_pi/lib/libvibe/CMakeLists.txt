aux_source_directory(. LIB_SRCS)
add_library(libvibe ${LIB_SRCS})
#lib name
set_target_properties(libvibe PROPERTIES OUTPUT_NAME "libvibe")  
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
