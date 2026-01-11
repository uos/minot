#!/bin/bash
# assuming sourced ros environment and starting from the package root directory
# creates the deb in the current dir

# needed for creating the deb
sudo apt-get install --no-install-recommends -y debhelper sed grep

# deps from the package.xml
sudo apt-get install --no-install-recommends -y cargo libclang-dev python3-catkin-pkg-modules

cargo_ver=$(grep -Po '^\s*version\s*=\s*"\K[^"]+' Cargo.toml)
xml_ver=$(grep -Po '<\s*version\s*>\K[^<]+' package.xml)

if [[ "$cargo_ver" != "$xml_ver" ]]; then
    echo "Versions differ!" >&2
    echo "   Cargo.toml  : $cargo_ver" >&2
    echo "   package.xml : $xml_ver" >&2
    exit 1
fi

shopt -s dotglob nullglob
mkdir minot-src
mv -- * minot-src/
shopt -u dotglob nullglob
tar -cJf "ros-$ROS_DISTRO-minot_$xml_ver.orig.tar.xz" minot-src

cd minot-src

bloom-generate rosdebian
dpkg-buildpackage -us -uc

