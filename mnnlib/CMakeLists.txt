# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.18.1)

# Declares and names the project.
set(LIB_NAME "mnnlib")
set(SRC_DIR src/main/cpp/)
set(OPENCV_LIB_DIR ../third/opencv/lib/${ANDROID_ABI}/)
set(OPENCV_3RD_LIB_DIR ../third/opencv/3rdparty/${ANDROID_ABI}/)
set(MNN_LIB_DIR ../third/mnn/lib/${ANDROID_ABI}/)

project(${LIB_NAME})

include_directories(../third/opencv/include
        src/main/cpp/include
        ../third/mnn/include
        ../third/mnn/include/MNN
        )

link_directories(${OPENCV_LIB_DIR}
        ${OPENCV_3RD_LIB_DIR}
        ${MNN_LIB_DIR}
        )

aux_source_directory(${SRC_DIR} DIR_LIB_SOURCE)
add_library(
        ${LIB_NAME}
         SHARED
        ${DIR_LIB_SOURCE})

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library(log-lib log )

target_link_libraries( # Specifies the target library.
        ${LIB_NAME}
        ${log-lib}
        MNN MNN_Express
        log jnigraphics z
        )

if(${ANDROID_ABI} STREQUAL x86 OR ${ANDROID_ABI} STREQUAL x86_64)
    target_link_libraries(${LIB_NAME} opencv_imgproc opencv_core ippiw ippicv ittnotify tbb cpufeatures)
else()
    target_link_libraries(${LIB_NAME} opencv_imgproc opencv_core tegra_hal tbb cpufeatures)
endif()
