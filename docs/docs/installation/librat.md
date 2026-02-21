# librat

Nodes in the Minot network that share data are called Rats ([here is why](../lore.md)). The functionality is shipped as a Rust and C library.

### Ubuntu

Our PPA provides `.deb` files for a system-wide installation. After the [setup](https://codeberg.org/uos-robotics/ppa/src/branch/pages/README.md), you can simply run apt.

~~~bash
sudo apt install librat-dev
~~~

### Debian-based Distros

The PPA mentioned above is specific to Ubuntu the package itself does not require any system dependencies. Therefore it can be installed manually on all debian-based distros.

~~~bash title="Manual .deb Installation"
curl -s https://api.github.com/repos/uos/minot/releases/latest \
| grep "browser_download_url" \
| grep ".deb" \
| grep "$(dpkg --print-architecture)" \
| cut -d '"' -f 4 \
| xargs curl -L -O

sudo dpkg -i ./librat-dev_*.deb
~~~

The package also installs a pkg-config file, which allows the following usage in CMake.

~~~cmake title="Example CMake"
find_package(PkgConfig REQUIRED)
pkg_check_modules(RAT REQUIRED librat)

add_executable(my_app main.c)
target_include_directories(my_app PRIVATE ${RAT_INCLUDE_DIRS})
target_link_libraries(myfind_package(PkgConfig REQUIRED)
pkg_check_modules(RAT REQUIRED librat)

add_executable(my_app main.c)
target_include_directories(my_app PRIVATE ${RAT_INCLUDE_DIRS})
target_link_libraries(my_app PRIVATE ${RAT_LIBRARIES})
~~~

### From Source

Building from source generates a static and shared library in the `./target/release/` folder. You will need to clone the repository first.

~~~bash title="Build librat from source"
git clone https://github.com/uos/minot
cd minot
cargo build --package mt_rat --release
~~~

A typical system-wide installation is done by copying the libraries to your linker path. Alternatively, you may change the link path and include search paths in your build system.

~~~bash
sudo cp ./target/release/librat.* /usr/local/lib/
sudo mkdir -p /usr/local/include/rat/
sudo cp ./rat/rat.h /usr/local/include/rat/
~~~

Then you can use the library in your C/C++ code.
~~~C
#include <rat/rat.h>
~~~

And link with `-lrat`.

---

For using the Rust library, just add this to your dependencies in `Cargo.toml`.

~~~toml title="Cargo.toml"
[dependencies]
mt_rat = "0.5.3"
~~~

