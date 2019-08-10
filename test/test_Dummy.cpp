#include <boost/test/unit_test.hpp>
#include <gps_ublox/Dummy.hpp>

using namespace gps_ublox;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    gps_ublox::DummyClass dummy;
    dummy.welcome();
}
