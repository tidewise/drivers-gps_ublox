#include <gps_ublox/Driver.hpp>
#include <iostream>

using namespace std;
using namespace gps_ublox;

Driver::Driver() : iodrivers_base::Driver(BUFFER_SIZE)
{
}
