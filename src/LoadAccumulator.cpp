#include "navion/dynamics/LoadAccumulator.hpp"

#include <stdexcept>

namespace navion
{

void LoadAccumulator::reset() noexcept
{
    total_ = {};
    contributingLoadCount_ = 0U;
}

void LoadAccumulator::add(const BodyLoad& load)
{
    if (!load.isFinite())
    {
        throw std::invalid_argument(
            "LoadAccumulator cannot add a non-finite BODY-axis load."
        );
    }

    total_ += load;
    ++contributingLoadCount_;
}

const BodyLoad& LoadAccumulator::total() const noexcept
{
    return total_;
}

std::size_t LoadAccumulator::contributingLoadCount() const noexcept
{
    return contributingLoadCount_;
}

} // namespace navion

