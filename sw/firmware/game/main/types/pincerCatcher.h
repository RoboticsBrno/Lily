#pragma once

#include <concepts>


template<typename PincerCatcher>
concept isPincerCatcher = requires(PincerCatcher pincerCatcher) {
    pincerCatcher.setPos(0);
    pincerCatcher.setPos(100);

    pincerCatcher.open();

    { pincerCatcher.getPos() } -> std::same_as<int>;
};
