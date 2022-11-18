# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/samuel/esp/esp-idf/components/bootloader/subproject"
  "/home/samuel/esp/projects/Projeto-Gangorra/build/bootloader"
  "/home/samuel/esp/projects/Projeto-Gangorra/build/bootloader-prefix"
  "/home/samuel/esp/projects/Projeto-Gangorra/build/bootloader-prefix/tmp"
  "/home/samuel/esp/projects/Projeto-Gangorra/build/bootloader-prefix/src/bootloader-stamp"
  "/home/samuel/esp/projects/Projeto-Gangorra/build/bootloader-prefix/src"
  "/home/samuel/esp/projects/Projeto-Gangorra/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/samuel/esp/projects/Projeto-Gangorra/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
