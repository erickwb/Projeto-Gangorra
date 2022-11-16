# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/pedrobotelho15/esp/esp-idf/components/bootloader/subproject"
  "/home/pedrobotelho15/esp/projects/projeto_gangorra_bluetooth/build/bootloader"
  "/home/pedrobotelho15/esp/projects/projeto_gangorra_bluetooth/build/bootloader-prefix"
  "/home/pedrobotelho15/esp/projects/projeto_gangorra_bluetooth/build/bootloader-prefix/tmp"
  "/home/pedrobotelho15/esp/projects/projeto_gangorra_bluetooth/build/bootloader-prefix/src/bootloader-stamp"
  "/home/pedrobotelho15/esp/projects/projeto_gangorra_bluetooth/build/bootloader-prefix/src"
  "/home/pedrobotelho15/esp/projects/projeto_gangorra_bluetooth/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pedrobotelho15/esp/projects/projeto_gangorra_bluetooth/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
