alias b := build
alias c := clean
alias i := install
alias u := uninstall

set working-directory := './software/opendulum_sw_proj'

set shell := ["powershell.exe", "/c"]

build_folder_path := absolute_path('build')

clean_command_string := if path_exists(build_folder_path) == "true" {
    'Remove-Item -Path "./build" -Recurse -Force'
} else {
    'echo "nothing to clean"'
}

build:
    cmake -DCMAKE_TOOLCHAIN_FILE="opendulum.cmake" -S ./ -B build -G"Unix Makefiles" 
    make -C build VERBOSE=1 -j

clean:
    {{clean_command_string}}

install:
    git submodule add -f -b "master" 'https://github.com/STMicroelectronics/stm32g4xx_hal_driver.git' './source/vendor/STM32G4xx_HAL_Driver' 
    git submodule add -f -b "master" 'https://github.com/STMicroelectronics/cmsis_device_g4.git' './source/vendor/cmsis_device_g4' 

uninstall:
    git rm './source/vendor/STM32G4xx_HAL_Driver' -f
    git rm './source/vendor/cmsis_device_g4' -f

