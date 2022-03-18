#include <gtest/gtest.h>
#include <gps_ublox/cfg.hpp>

using namespace gps_ublox;

struct cfgTest : public ::testing::Test {
};

TEST_F(cfgTest, getRTCMOutputKey_returns_a_RTCM_output_configuration_key) {
    ASSERT_EQ(0x2091035f, cfg::getRTCMOutputKey(PORT_UART1, 1074));
}

TEST_F(cfgTest, getRTCMOutputKey_returns_the_subtype_zero_for_4072) {
    ASSERT_EQ(0x209102ff, cfg::getRTCMOutputKey(PORT_UART1, 4072));
}

TEST_F(cfgTest, getRTCMOutputKey_allows_to_explicitly_select_4072_variant_0) {
    ASSERT_EQ(0x209102ff, cfg::getRTCMOutputKey(PORT_UART1, 40720));
}

TEST_F(cfgTest, getRTCMOutputKey_allows_to_explicitly_select_4072_variant_1) {
    ASSERT_EQ(0x20910382, cfg::getRTCMOutputKey(PORT_UART1, 40721));
}