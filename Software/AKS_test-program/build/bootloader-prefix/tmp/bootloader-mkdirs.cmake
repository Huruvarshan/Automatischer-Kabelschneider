# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/e1Suresh/Documents/v5.2.2/esp-idf/components/bootloader/subproject"
  "C:/Users/e1Suresh/Documents/GitHub/Automatischer-Kabelschneider/Software/AKS_test-program/build/bootloader"
  "C:/Users/e1Suresh/Documents/GitHub/Automatischer-Kabelschneider/Software/AKS_test-program/build/bootloader-prefix"
  "C:/Users/e1Suresh/Documents/GitHub/Automatischer-Kabelschneider/Software/AKS_test-program/build/bootloader-prefix/tmp"
  "C:/Users/e1Suresh/Documents/GitHub/Automatischer-Kabelschneider/Software/AKS_test-program/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/e1Suresh/Documents/GitHub/Automatischer-Kabelschneider/Software/AKS_test-program/build/bootloader-prefix/src"
  "C:/Users/e1Suresh/Documents/GitHub/Automatischer-Kabelschneider/Software/AKS_test-program/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/e1Suresh/Documents/GitHub/Automatischer-Kabelschneider/Software/AKS_test-program/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/e1Suresh/Documents/GitHub/Automatischer-Kabelschneider/Software/AKS_test-program/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
