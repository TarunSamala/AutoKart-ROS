#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
vendor_dir="$script_dir/vendor"
sdk_dir="$vendor_dir/linux_ros"
package_file="$vendor_dir/linux_ros.pkg"
archive_file="$vendor_dir/linux_ros.tar.xz"
package_url="https://raw.githubusercontent.com/Drone-SWAN/YDLIDAR-ICA-ASC60C/482a43868acfdf2b6d64dc1a9c3560af50beb44c/Ubuntu-24.04-Headless/linux_ros.pkg"
package_sha256="24a71680e96411d3cc9217654b93d492f5afbfe212b8c01e987371d26792f404"

for command_name in curl sha256sum tar xz patch; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "Missing required command: $command_name" >&2
        exit 1
    fi
done

if [[ -d "$sdk_dir/linux" ]]; then
    echo "HP60C SDK is already prepared in $sdk_dir"
    exit 0
fi

mkdir -p "$vendor_dir"

if [[ ! -f "$package_file" ]]; then
    echo "Downloading HP60C Linux SDK package..."
    curl -L --fail --show-error --progress-bar -o "$package_file" "$package_url"
fi

echo "$package_sha256  $package_file" | sha256sum --check --status || {
    echo "HP60C SDK checksum verification failed; remove $package_file and retry." >&2
    exit 1
}

echo "Extracting HP60C Linux SDK..."
{
    printf '\xFD\x37\x7A\x58\x5A'
    dd if="$package_file" status=none
} >"$archive_file"
tar -xJf "$archive_file" -C "$vendor_dir"
rm -f "$archive_file"

cp -a "$sdk_dir/libs" "$sdk_dir/configurationfiles" "$sdk_dir/scripts" "$sdk_dir/linux/"

patch --forward --directory="$sdk_dir" -p1 <"$script_dir/vendor-compatibility.patch"

echo "HP60C SDK setup complete. Build it with: ./build.sh"
