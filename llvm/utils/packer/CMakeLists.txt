cmake_minimum_required(VERSION 3.20.0)
project(packer NONE)

# Install pack.py with execute permission
install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/pack.py
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/llvm/packer
        PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_READ WORLD_READ
)

# Install all files except CPython binaries
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/llvm/packer/
        PATTERN "cpython/bin" EXCLUDE
)

# Install CPython binaries preserving execute permission
file(GLOB CPYTHON_BINARIES "${CMAKE_CURRENT_SOURCE_DIR}/cpython/bin/*")
install(
        PROGRAMS ${CPYTHON_BINARIES}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/llvm/packer/cpython/bin
)

# Install rest of CPython directory (excluding bin)
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cpython/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/llvm/packer/cpython/
        PATTERN "bin" EXCLUDE
)
