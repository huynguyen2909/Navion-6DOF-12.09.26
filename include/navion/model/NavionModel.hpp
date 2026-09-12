#pragma once

#include "navion/components/IGroundContactComponent.hpp"
#include "navion/components/ILoadComponent.hpp"
#include "navion/dynamics/RigidBody6DOF.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace navion
{

class NavionModel final
{
public:
    explicit NavionModel(const MassProperties& massProperties);

    void addLoadComponent(std::unique_ptr<ILoadComponent> component);
    void setGroundContactComponent(
        std::unique_ptr<IGroundContactComponent> component
    );

    [[nodiscard]] ModelEvaluation evaluate(
        double timeS,
        const RigidBodyState& state,
        const ControlInputs& controls,
        const Environment& environment
    ) const;

    // Coupled evaluation used when a ground-contact component is installed.
    // The same outer-step dt is supplied at every RK4 stage because the PGS
    // stabilization term contains velocity/dt.
    [[nodiscard]] ModelEvaluation evaluate(
        double timeS,
        double timeStepS,
        const RigidBodyState& state,
        const ControlInputs& controls,
        const Environment& environment
    ) const;

    [[nodiscard]] StateDerivative evaluateDerivative(
        double timeS,
        const RigidBodyState& state,
        const ControlInputs& controls,
        const Environment& environment
    ) const;

    [[nodiscard]] StateDerivative evaluateDerivative(
        double timeS,
        double timeStepS,
        const RigidBodyState& state,
        const ControlInputs& controls,
        const Environment& environment
    ) const;

    // Commits contact/slip and PGS warm-start history once, after an outer
    // integration step has been accepted. Derivative evaluations never
    // mutate this history.
    void commitAcceptedStep(
        double timeS,
        double timeStepS,
        const RigidBodyState& acceptedState,
        const ControlInputs& controls,
        const Environment& environment
    );

    void resetGroundContactHistory() noexcept;

    [[nodiscard]] std::size_t componentCount() const noexcept;
    [[nodiscard]] bool hasGroundContactComponent() const noexcept;
    [[nodiscard]] const MassProperties& massProperties() const noexcept;

private:
    struct CoupledEvaluation;

    [[nodiscard]] CoupledEvaluation evaluateCoupled(
        double timeS,
        double timeStepS,
        const RigidBodyState& state,
        const ControlInputs& controls,
        const Environment& environment
    ) const;

    [[nodiscard]] static FlightCondition calculateFlightCondition(
        const RigidBodyState& state,
        const Environment& environment
    );

    RigidBody6DOF rigidBody_;
    std::vector<std::unique_ptr<ILoadComponent>> loadComponents_{};
    std::unique_ptr<IGroundContactComponent> groundContactComponent_{};
};

} // namespace navion
