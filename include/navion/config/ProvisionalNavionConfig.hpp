#pragma once

#include "navion/components/fuselage/FuselageComponent.hpp"
#include "navion/components/landing_gear/LandingGearComponent.hpp"
#include "navion/components/propeller/PropellerModel.hpp"
#include "navion/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include "navion/components/stabilizers/VATC_VerticalStabilizer.hpp"
#include "navion/components/wings/VATC_MainWing.hpp"
#include "navion/model/NavionModel.hpp"

namespace navion
{

// One explicit assembly point for the current Navion-oriented V1 estimate.
// The type name says "Provisional" because some source data are surrogates;
// see docs/MODEL_DATA_STATUS.md before interpreting performance results.
struct ProvisionalNavionConfig
{
    MassProperties massProperties{};
    MainWingConfig mainWing{};
    HorizontalStabilizerConfig horizontalStabilizer{};
    VerticalStabilizerConfig verticalStabilizer{};
    fuselage::FuselageComponentConfig fuselage{};
    propeller::PropellerParameters propeller{};
    landing_gear::LandingGearParameters landingGear{};
};

[[nodiscard]] ProvisionalNavionConfig makeProvisionalNavionConfig();

// Registers five ordinary load components and the coupled Landing Gear.
// NavionModel keeps ownership through its existing polymorphic contracts.
void addProvisionalNavionComponents(
    NavionModel& model,
    const ProvisionalNavionConfig& config
);

} // namespace navion
