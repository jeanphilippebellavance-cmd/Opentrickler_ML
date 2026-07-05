param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType = "Release",
    [string]$BuildDir = "build-pico2w-release"
)

. .\configure_env.ps1

# Configure CMake for Raspberry Pi Pico 2 W / RP2350
cmake -B $BuildDir -G Ninja -DCMAKE_BUILD_TYPE=$BuildType -DPICO_BOARD=pico2_w
# cmake --build $BuildDir --config $BuildType
