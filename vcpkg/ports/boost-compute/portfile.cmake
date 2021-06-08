# Automatically generated by scripts/boost/generate-ports.ps1

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO boostorg/compute
    REF boost-1.75.0
    SHA512 740e351c466d133058b2730311194198f958504b6b0c20517e54ad7809d71d3fe11d35b7935d6bf49f191d4dea7a9f37be21fa4f8d363331588a32d73febc047
    HEAD_REF master
)

include(${CURRENT_INSTALLED_DIR}/share/boost-vcpkg-helpers/boost-modular-headers.cmake)
boost_modular_headers(SOURCE_PATH ${SOURCE_PATH})