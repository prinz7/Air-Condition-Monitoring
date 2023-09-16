#include <iostream>
#include "bsec_datatypes.h"
#include "bsec_interface.h"

int main() {
    auto x = bsec_init();
    if (x == bsec_library_return_t::BSEC_OK)
    {
        std::cout << "Init success!" << std::endl;
    }
    else
    {
        std::cout << "Init failed: " << x << std::endl;
    }

    return 0;
}
