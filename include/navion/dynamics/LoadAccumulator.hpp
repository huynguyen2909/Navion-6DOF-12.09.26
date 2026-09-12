#pragma once

#include "navion/core/FlightTypes.hpp"

#include <cstddef>

namespace navion
{

class LoadAccumulator final
{
public:
    void reset() noexcept;
    void add(const BodyLoad& load);

    [[nodiscard]] const BodyLoad& total() const noexcept;
    [[nodiscard]] std::size_t contributingLoadCount() const noexcept;

private:
    BodyLoad total_{};
    std::size_t contributingLoadCount_{0U};
};

} // namespace navion

