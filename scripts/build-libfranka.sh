#!/bin/bash

cd ~/libs
sudo apt install -y libpoco-dev libeigen3-dev
git clone https://github.com/frankaemika/libfranka.git --recursive
cd libfranka
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  ..
cmake --build . -j$(nproc)
cpack -G DEB
sudo dpkg -i libfranka-*.deb
