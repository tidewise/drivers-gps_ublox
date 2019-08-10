#include <gps_ublox/Driver.hpp>
#include <iostream>

using namespace std;
using namespace gps_ublox;

Driver::Driver() : iodrivers_base::Driver(BUFFER_SIZE)
{
}

int Driver::extractPacket(const uint8_t *buffer, size_t buffer_size) const
{
    return -1;
}
