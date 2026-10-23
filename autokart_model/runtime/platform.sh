#!/usr/bin/env bash

# Return the vendor SDK directory matching the compiler used for this build.
# Raspberry Pi OS may be either 64-bit (aarch64) or 32-bit (hard-float ARM).
get_sdk_lib_target() {
    local compiler_target
    compiler_target=$(g++ -dumpmachine)

    case "$compiler_target" in
        x86_64*) echo "x86_64-linux-gnu" ;;
        aarch64*) echo "aarch64-linux-gnu" ;;
        arm*gnueabihf|armv7*) echo "arm-linux-gnueabihf" ;;
        *)
            echo "Unsupported compiler target: $compiler_target" >&2
            return 1
            ;;
    esac
}

get_build_dir() {
    local source_dir=$1
    local sdk_lib_target
    sdk_lib_target=$(get_sdk_lib_target)
    echo "$source_dir/build-hp60c-$sdk_lib_target"
}
