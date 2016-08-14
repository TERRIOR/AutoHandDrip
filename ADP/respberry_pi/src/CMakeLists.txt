#lib fiel name
include_directories(${PROJECT_SOURCE_DIR}/lib/libvibe)
cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED) 
#all file take part in
aux_source_directory(. APP_SRC)
#project name
add_executable(vibefite ${APP_SRC})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#project name +lib name
target_link_libraries(vibefite libvibe ${OpenCV_LIBS})
