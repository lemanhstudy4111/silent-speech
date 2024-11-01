# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/assignment1/Assignment_Final/IMU/build/bootloader"
  "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/assignment1/Assignment_Final/IMU/build/bootloader-prefix"
  "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/assignment1/Assignment_Final/IMU/build/bootloader-prefix/tmp"
  "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/assignment1/Assignment_Final/IMU/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/assignment1/Assignment_Final/IMU/build/bootloader-prefix/src"
  "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/assignment1/Assignment_Final/IMU/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/assignment1/Assignment_Final/IMU/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/honga/Documents/UMASS AMHERST/Fall 2024/CS528/assignment1/Assignment_Final/IMU/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
