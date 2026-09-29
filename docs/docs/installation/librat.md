# librat

Nodes in the Minot network that share data are called Rats ([here is why](../lore.md)). The functionality is shipped as a Rust and C library.

### Ubuntu

Our PPA provides `.deb` files for a system-wide installation.

~~~bash title="steado PPA"
curl -fsSL "https://ppa.steado.tech/ubuntu/key.gpg" | gpg --dearmor \
  | sudo tee /usr/share/keyrings/steado-archive-keyring.gpg >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/steado-archive-keyring.gpg] https://ppa.steado.tech/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/steado.list
sudo apt update
~~~

After the setup, you can simply run apt.

~~~bash
sudo apt install librat-dev
~~~

### Debian-based Distros

The PPA mentioned above is specific to Ubuntu the package itself does not require any system dependencies. Therefore it can be installed manually on all debian-based distros.

~~~bash title="Manual .deb Installation"
curl -s https://codeberg.org/api/v1/repos/stelzo/minot/releases/latest \
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

### Prebuilt Files

Every [release](archives.md) contains `librat-<target>.a`, `librat-<target>.so` or `librat-<target>.dylib`, `rat.h`, `librat.pc` and `libratConfig.cmake`. The [installation script](script.md) installs them with `--with-rat`.

~~~bash
curl -sSLf https://stelzo.codeberg.page/minot/install | sh -s -- --with-rat
~~~

### From Source

Building from source generates a static and shared library in the `./target/full-release/` folder. You will need to clone the repository first.

~~~bash title="Build librat from source"
git clone https://codeberg.org/stelzo/minot
cd minot
cargo rustc --package mt_rat --profile full-release --lib --crate-type staticlib,cdylib
~~~

Restricting the build to the C library types lets the `full-release` profile apply link-time optimization, which keeps the static library small. A plain `cargo build --package mt_rat --release` also works, but its static library is several times larger.

A typical system-wide installation is done by copying the libraries to your linker path. Alternatively, you may change the link path and include search paths in your build system.

~~~bash
sudo cp ./target/full-release/librat.* /usr/local/lib/
sudo mkdir -p /usr/local/include/rat/
sudo cp ./mt_rat/rat.h /usr/local/include/rat/
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
mt_rat = "0.9.0"
~~~

