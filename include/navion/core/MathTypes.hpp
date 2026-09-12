#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace navion
{

struct Vec3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double xValue, double yValue, double zValue)
        : x(xValue), y(yValue), z(zValue)
    {
    }

    [[nodiscard]] bool isFinite() const noexcept
    {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    [[nodiscard]] constexpr double normSquared() const noexcept
    {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] double norm() const noexcept
    {
        return std::sqrt(normSquared());
    }

    Vec3& operator+=(const Vec3& rhs) noexcept
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs) noexcept
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3& operator*=(double scalar) noexcept
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
};

[[nodiscard]] constexpr Vec3 operator+(Vec3 lhs, const Vec3& rhs) noexcept
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

[[nodiscard]] constexpr Vec3 operator-(Vec3 lhs, const Vec3& rhs) noexcept
{
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;
    return lhs;
}

[[nodiscard]] constexpr Vec3 operator-(const Vec3& value) noexcept
{
    return {-value.x, -value.y, -value.z};
}

[[nodiscard]] constexpr Vec3 operator*(Vec3 value, double scalar) noexcept
{
    value.x *= scalar;
    value.y *= scalar;
    value.z *= scalar;
    return value;
}

[[nodiscard]] constexpr Vec3 operator*(double scalar, Vec3 value) noexcept
{
    return value * scalar;
}

[[nodiscard]] inline Vec3 operator/(Vec3 value, double scalar)
{
    if (scalar == 0.0 || !std::isfinite(scalar))
    {
        throw std::invalid_argument("Vec3 division requires a finite non-zero scalar.");
    }
    return value * (1.0 / scalar);
}

[[nodiscard]] constexpr double dot(const Vec3& lhs, const Vec3& rhs) noexcept
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

[[nodiscard]] constexpr Vec3 cross(const Vec3& lhs, const Vec3& rhs) noexcept
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

class Matrix3
{
public:
    constexpr Matrix3() = default;

    constexpr explicit Matrix3(const std::array<double, 9>& values)
        : values_(values)
    {
    }

    [[nodiscard]] static constexpr Matrix3 identity() noexcept
    {
        return Matrix3({
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        });
    }

    [[nodiscard]] static constexpr Matrix3 diagonal(
        double xx,
        double yy,
        double zz
    ) noexcept
    {
        return Matrix3({
            xx, 0.0, 0.0,
            0.0, yy, 0.0,
            0.0, 0.0, zz
        });
    }

    [[nodiscard]] constexpr double operator()(
        std::size_t row,
        std::size_t column
    ) const noexcept
    {
        return values_[row * 3U + column];
    }

    [[nodiscard]] bool isFinite() const noexcept
    {
        return std::all_of(
            values_.begin(),
            values_.end(),
            [](double value) { return std::isfinite(value); }
        );
    }

    [[nodiscard]] bool isSymmetric(double tolerance = 1.0e-12) const noexcept
    {
        return
            std::abs((*this)(0, 1) - (*this)(1, 0)) <= tolerance &&
            std::abs((*this)(0, 2) - (*this)(2, 0)) <= tolerance &&
            std::abs((*this)(1, 2) - (*this)(2, 1)) <= tolerance;
    }

    [[nodiscard]] constexpr Vec3 operator*(const Vec3& vector) const noexcept
    {
        return {
            (*this)(0, 0) * vector.x + (*this)(0, 1) * vector.y + (*this)(0, 2) * vector.z,
            (*this)(1, 0) * vector.x + (*this)(1, 1) * vector.y + (*this)(1, 2) * vector.z,
            (*this)(2, 0) * vector.x + (*this)(2, 1) * vector.y + (*this)(2, 2) * vector.z
        };
    }

    [[nodiscard]] constexpr Matrix3 transposed() const noexcept
    {
        return Matrix3({
            (*this)(0, 0), (*this)(1, 0), (*this)(2, 0),
            (*this)(0, 1), (*this)(1, 1), (*this)(2, 1),
            (*this)(0, 2), (*this)(1, 2), (*this)(2, 2)
        });
    }

    [[nodiscard]] constexpr double determinant() const noexcept
    {
        const double a = (*this)(0, 0);
        const double b = (*this)(0, 1);
        const double c = (*this)(0, 2);
        const double d = (*this)(1, 0);
        const double e = (*this)(1, 1);
        const double f = (*this)(1, 2);
        const double g = (*this)(2, 0);
        const double h = (*this)(2, 1);
        const double i = (*this)(2, 2);

        return
            a * (e * i - f * h) -
            b * (d * i - f * g) +
            c * (d * h - e * g);
    }

    [[nodiscard]] Matrix3 inverse(double tolerance = 1.0e-14) const
    {
        const double determinantValue = determinant();
        if (!std::isfinite(determinantValue) ||
            std::abs(determinantValue) <= tolerance)
        {
            throw std::invalid_argument("Matrix3 is singular or non-finite.");
        }

        const double a = (*this)(0, 0);
        const double b = (*this)(0, 1);
        const double c = (*this)(0, 2);
        const double d = (*this)(1, 0);
        const double e = (*this)(1, 1);
        const double f = (*this)(1, 2);
        const double g = (*this)(2, 0);
        const double h = (*this)(2, 1);
        const double i = (*this)(2, 2);

        const double inverseDeterminant = 1.0 / determinantValue;
        return Matrix3({
            (e * i - f * h) * inverseDeterminant,
            (c * h - b * i) * inverseDeterminant,
            (b * f - c * e) * inverseDeterminant,
            (f * g - d * i) * inverseDeterminant,
            (a * i - c * g) * inverseDeterminant,
            (c * d - a * f) * inverseDeterminant,
            (d * h - e * g) * inverseDeterminant,
            (b * g - a * h) * inverseDeterminant,
            (a * e - b * d) * inverseDeterminant
        });
    }

private:
    std::array<double, 9> values_{
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };
};

struct Quaternion
{
    // Hamilton convention, scalar first. q_nb rotates BODY components to NED.
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return
            std::isfinite(w) && std::isfinite(x) &&
            std::isfinite(y) && std::isfinite(z);
    }

    [[nodiscard]] constexpr double normSquared() const noexcept
    {
        return w * w + x * x + y * y + z * z;
    }

    [[nodiscard]] Quaternion normalized(double tolerance = 1.0e-14) const
    {
        const double magnitude = std::sqrt(normSquared());
        if (!std::isfinite(magnitude) || magnitude <= tolerance)
        {
            throw std::invalid_argument("Quaternion must have a finite non-zero norm.");
        }

        const double inverseMagnitude = 1.0 / magnitude;
        return {
            w * inverseMagnitude,
            x * inverseMagnitude,
            y * inverseMagnitude,
            z * inverseMagnitude
        };
    }

    [[nodiscard]] constexpr Quaternion conjugate() const noexcept
    {
        return {w, -x, -y, -z};
    }

    [[nodiscard]] Vec3 rotate(const Vec3& vector) const
    {
        const Quaternion unit = normalized();
        const Vec3 qVector{unit.x, unit.y, unit.z};
        const Vec3 intermediate = 2.0 * cross(qVector, vector);
        return vector + unit.w * intermediate + cross(qVector, intermediate);
    }

    Quaternion& operator+=(const Quaternion& rhs) noexcept
    {
        w += rhs.w;
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
};

[[nodiscard]] constexpr Quaternion operator*(
    const Quaternion& lhs,
    const Quaternion& rhs
) noexcept
{
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w
    };
}

[[nodiscard]] constexpr Quaternion operator+(
    Quaternion lhs,
    const Quaternion& rhs
) noexcept
{
    lhs.w += rhs.w;
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

[[nodiscard]] constexpr Quaternion operator*(
    Quaternion value,
    double scalar
) noexcept
{
    value.w *= scalar;
    value.x *= scalar;
    value.y *= scalar;
    value.z *= scalar;
    return value;
}

[[nodiscard]] constexpr Quaternion operator*(
    double scalar,
    Quaternion value
) noexcept
{
    return value * scalar;
}

[[nodiscard]] inline Quaternion quaternionDerivativeBodyToNed(
    const Quaternion& attitudeBodyToNed,
    const Vec3& angularRateBodyRadps
)
{
    const Quaternion unitAttitude = attitudeBodyToNed.normalized();
    const Quaternion bodyRate{
        0.0,
        angularRateBodyRadps.x,
        angularRateBodyRadps.y,
        angularRateBodyRadps.z
    };
    return 0.5 * (unitAttitude * bodyRate);
}

} // namespace navion
