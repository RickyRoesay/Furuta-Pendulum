#############################################################################################################################
# file:  CMakeLists.txt
# brief: Template "CMakeLists.txt" for building of executables and static libraries.
#
# usage: Edit "VARIABLES"-section to suit project requirements.
#        For debug build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
#          make -C Debug VERBOSE=1 -j
#        For release build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Release -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
#          make -C Release VERBOSE=1 -j
#
#		Dependencies: 
#			- CMake and Make
#			- arm-none-eabi-gcc installed via MSYS, by typing:   <pacman -S mingw-w64-x86_64-arm-none-eabi-gcc>   without the <>
#           - just, using winget, by typing   <winget install --id Casey.Just --exact>   into command line without the <> 
#
#  GIT SUBMODULE CMD MESSAGES
#  git submodule add -b "master" 'https://github.com/STMicroelectronics/stm32g4xx_hal_driver.git' 'software/opendulum_sw_proj/source/vendor/STM32G4xx_HAL_Driver'
#  git submodule add -b "master" 'https://github.com/STMicroelectronics/cmsis_device_g4.git' 'software/opendulum_sw_proj/source/vendor/cmsis_device_g4'
#
#
#
#############################################################################################################################
cmake_minimum_required(VERSION 3.20)

###################### CONSTANTS ######################################
set (PROJECT_TYPE_EXECUTABLE          "exe")
set (PROJECT_TYPE_STATIC_LIBRARY      "static-lib")
set (MCPU_CORTEX_M4				      "-mcpu=cortex-m4")
set (MFPU_FPV4_SP_D16                 "-mfpu=fpv4-sp-d16")
set (MFPU_FPV5_D16                    "-mfpu=fpv5-d16")
set (MFLOAT_ABI_SOFTWARE              "-mfloat-abi=soft")
set (MFLOAT_ABI_HARDWARE              "-mfloat-abi=hard")
set (MFLOAT_ABI_MIX                   "-mfloat-abi=softfp")
#######################################################################

#
#find_program(CMAKE_MAKE_PROGRAM
#  NAMES make
#        make.exe
#  DOC "Find a suitable make program for building under Windows/MinGW"
#  HINTS c:/MinGW-32/bin )  



###################### VARIABLES ######################################
set (PROJECT_NAME             "OPENDULUM_SW_PROJ")
set (PROJECT_TYPE             "exe")
set (LINKER_SCRIPT            "../STM32G474CEUX_FLASH.ld")
set (MCPU                     "-mcpu=Cortex-M4")
set (MFPU                     "-mfpu=fpv4-sp-d16")

set (STM32G4xx_HAL_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/source/vendor/STM32G4xx_HAL_Driver/Src")

set (PROJECT_SOURCES
	# LIST SOURCE FILES HERE
	source/vendor/startup_stm32g474ceux.s
	source/main.c
	${STM32G4xx_HAL_DIRECTORY}/stm32g4xx_hal_adc.c
	)


set (PROJECT_DEFINES
	# LIST COMPILER DEFINITIONS HERE
	STM32G474xx
	USE_TIMEOUT=1
    )


set (PROJECT_INCLUDES
	# LIST INCLUDE DIRECTORIES HERE
	${CMAKE_CURRENT_SOURCE_DIR}/source/vendor/cmsis_device_g4/Include
	${CMAKE_CURRENT_SOURCE_DIR}/source/vendor/cmsis_core/
	${CMAKE_CURRENT_SOURCE_DIR}/source/vendor/STM32G4xx_HAL_Driver/
	${CMAKE_CURRENT_SOURCE_DIR}/source/vendor/STM32G4xx_HAL_Driver/Inc/
    )


############ MODIFY ACCORDING TO REQUIREMENTS) ########################

#######################################################################

################## PROJECT SETUP ######################################
project(${PROJECT_NAME})
enable_language(ASM)

if (${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_EXECUTABLE})
  add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
  add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
elseif (${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_STATIC_LIBRARY})
  add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
endif()

add_compile_definitions (${PROJECT_DEFINES})
include_directories (${PROJECT_INCLUDES})

set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI}  -mthumb -Wall -Werror")
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
