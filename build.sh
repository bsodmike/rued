#!/bin/bash

set -e

WORKSPACE="rued-esp32"
version=$(sed -n 's/^version = //p' $WORKSPACE/Cargo.toml | tr -d '"')
echo "Build Version: $version"

if [ ! -d "./bin" ]
then
    echo "./bin missing; creating..."
    mkdir ./bin
fi

echo "Building release"
CARGO_PKG_VERSION=$version && cargo build --release

echo "Building firmware image"
CARGO_PKG_VERSION=$version && cargo espflash  save-image --release --package rued-esp32 --chip esp32 --target xtensa-esp32-espidf --flash-size 2M ./bin/firmware-$version.bin