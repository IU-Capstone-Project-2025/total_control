# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/artyom/esp/v5.4.1/esp-idf/components/bootloader/subproject"
  "/home/artyom/Personal/inno_study/total_control/firmware/motors_test/build/bootloader"
  "/home/artyom/Personal/inno_study/total_control/firmware/motors_test/build/bootloader-prefix"
  "/home/artyom/Personal/inno_study/total_control/firmware/motors_test/build/bootloader-prefix/tmp"
  "/home/artyom/Personal/inno_study/total_control/firmware/motors_test/build/bootloader-prefix/src/bootloader-stamp"
  "/home/artyom/Personal/inno_study/total_control/firmware/motors_test/build/bootloader-prefix/src"
  "/home/artyom/Personal/inno_study/total_control/firmware/motors_test/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/artyom/Personal/inno_study/total_control/firmware/motors_test/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/artyom/Personal/inno_study/total_control/firmware/motors_test/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
