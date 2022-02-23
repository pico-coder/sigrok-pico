Building the RP2040 device code should be nearly exact to building any other Raspberry Pi PICO C SDK build as described in the "getting-started-with-pico".
1) Setup the PICO C SDK and go through the example builds using cmake and make
2) git clone this repo to <repo_dir>.
3) cd <repo_dir>/sigrok-pico
4) copy pico_sdk_import.cmake from your pico-sdk repo to <repo_dir>/sigrok_pico
5) export PICO_SDK_PATH=<path_to_your_pico_sdk>
6) cd <repo_dir>/sigrok-pico
7) mkdir build
8) cd build
9) cmake ..
10) make
