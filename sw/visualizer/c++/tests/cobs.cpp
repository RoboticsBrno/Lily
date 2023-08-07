#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cobs.h>
#include <vector>
#include <span>
#include <tuple>

#include "util.h"


TEST_CASE("Encode-decode", "[cobs]") {

    using sgn = typename std::tuple<std::string, std::vector<uint8_t>>;
    auto [comment, data] = GENERATE(
        sgn{ "Test", {0x01, 0x02, 0x03, 0x00, 0x04, 0x05, 0x00, 0x06, 0x00} }
    );

    DYNAMIC_SECTION(comment) {
        std::vector<uint8_t> copy = data;
        copy.insert(copy.begin(), 0x00);
        copy.push_back(0x00);

        cobs_encode(std::span<uint8_t>(copy));

        auto decoded = cobs_decode(std::span<uint8_t>(copy));

        REQUIRE(decoded.size() == data.size());

        EQUAL_ITERABLE(data, decoded);
    }
}
