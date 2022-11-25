# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v4.4.2/components/bootloader/subproject"
  "C:/ASL/Universidad/4to_curso/SBC/TestGithub/SBC_22_M_01/simple/build/bootloader"
  "C:/ASL/Universidad/4to_curso/SBC/TestGithub/SBC_22_M_01/simple/build/bootloader-prefix"
  "C:/ASL/Universidad/4to_curso/SBC/TestGithub/SBC_22_M_01/simple/build/bootloader-prefix/tmp"
  "C:/ASL/Universidad/4to_curso/SBC/TestGithub/SBC_22_M_01/simple/build/bootloader-prefix/src/bootloader-stamp"
  "C:/ASL/Universidad/4to_curso/SBC/TestGithub/SBC_22_M_01/simple/build/bootloader-prefix/src"
  "C:/ASL/Universidad/4to_curso/SBC/TestGithub/SBC_22_M_01/simple/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/ASL/Universidad/4to_curso/SBC/TestGithub/SBC_22_M_01/simple/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
