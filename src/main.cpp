#include <iostream>
#include "bsec_datatypes.h"
#include "bsec_interface.h"

int main() {
    std::cout << "Hello, CMake project with C++20!" << std::endl;
    return 0;

    bsec_version_t s;
    auto x = bsec_init();
    if (x == bsec_library_return_t::BSEC_OK)
    {
        std::cout << "Init success!" << std::endl;
    }
    else
    {
        std::cout << "Init failed: " << x << std::endl;
    }
}
