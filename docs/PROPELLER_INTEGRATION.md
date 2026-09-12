# Tích hợp Propeller vào kiến trúc OOP Stage 3

## 1. Phân tách trách nhiệm

Propeller Stage 3 gồm hai lớp có vai trò khác nhau:

| Lớp | Vai trò |
| --- | --- |
| `propeller::PropellerModel` | Kernel BEMT, tra polar, tích phân disk và giải uniform inflow |
| `propeller::PropellerComponent` | Adapter aircraft-facing triển khai `ILoadComponent` |

Chỉ `PropellerComponent` được đăng ký với `NavionModel`. Kernel vẫn có
`RuntimeInput` và `PropellerOutput` để unit test, xuất blade-element diagnostics
và kiểm tra BEMT độc lập.

## 2. Ánh xạ từ contract chung

Trong mỗi lần `computeLoad(context)`:

```text
context.flightCondition.airRelativeVelocityBodyMps
    -> RuntimeInput.velocityCgRelativeAirBodyMps

context.state.angularRateBodyRadps
    -> RuntimeInput.angularRateBodyWrtInertialBodyRadps

context.environment.airDensityKgM3
    -> RuntimeInput.airDensityKgM3

context.environment.speedOfSoundMps
    -> RuntimeInput.speedOfSoundMps

context.controls.propellerSpeedScale
    -> RuntimeInput.rotationRateScale

context.controls.propellerEnabled
    -> RuntimeInput.enabled
```

Uniform atmosphere wind đã được `NavionModel` đưa vào aircraft-relative
velocity. Propeller không trừ wind lần thứ hai.

## 3. Đầu ra tải

Sau khi inflow hội tụ:

```text
BodyLoad.forceBodyN          = PropellerOutput.forceBodyN
BodyLoad.momentAboutCgBodyNm = PropellerOutput.totalMomentAtCgBodyNm
```

Moment tổng đã gồm:

```text
M_prop,CG = M_aero,hub + r_CG_to_hub × F_prop + M_gyro
```

Trong đó `M_aero,hub` đã chứa cả aerodynamic reaction torque dọc shaft và hub
bending moments do oblique inflow/P-factor/rate effects. Vì vậy:

- không cộng reaction torque lần thứ hai;
- không tính lại `r × F` trong `LoadAccumulator`;
- không cộng gyroscopic moment lần thứ hai trong `RigidBody6DOF`.

## 4. Inflow trong RK4

`lambdaInduced` là nghiệm đại số quasi-steady, không phải state động lực học.
Mỗi k1–k4 có state và local hub velocity khác nhau, vì vậy mỗi stage chạy lại:

```text
hub kinematics -> inflow residual -> BEMT -> converged disk load
```

`PropellerComponent` không giữ `lastLambdaInduced`. Mỗi evaluation bắt đầu từ
`PropellerParameters::inflow.initialGuess`, sau đó safeguarded Newton/bisection
tự bracket và hội tụ. Cách này deterministic, không phụ thuộc thứ tự gọi và
không làm k1 rò sang k2–k4.

`PropellerModel::evaluate(input, previousLambdaInduced)` vẫn tồn tại cho công
cụ chẩn đoán độc lập muốn truyền một numerical guess tường minh. Giá trị đó
không được lưu trong component và không đi vào state vector.

Nếu solver không hội tụ, kernel trả candidate tốt nhất kèm diagnostics, nhưng
`PropellerComponent::computeLoad()` ném `runtime_error`; tải chưa hội tụ không
được cộng vào rigid-body equations.

## 5. Prescribed RPM, fixed pitch và throttle

Stage 5.3 giữ kernel BEMT của Propeller V1 và bổ sung tốc độ quay tức thời:

- `PropellerParameters::rotationRateRadps` là tốc độ cấu hình 2300 RPM;
- `ControlInputs::propellerSpeedScale` là hệ số không âm, được đánh giá lại ở
  từng stage RK4;
- blade pitch distribution cố định;
- không có engine power balance, governor dynamics hoặc pitch actuator;
- `ControlInputs::throttle` chưa thay đổi thrust/torque;
- `ControlInputs::propellerEnabled` vẫn bật/tắt toàn bộ propeller model.

Khi speed scale bằng 0, kernel trả về một nghiệm zero-load hợp lệ và không
chia cho tip speed bằng 0. Scenario Stage 5.3 giữ RPM bằng 0 trong 2 s, ramp
smoothstep trong 5 s và giữ 2300 RPM đến t=30 s.

Không scale lực tuyến tính theo throttle vì cách đó không bảo toàn quan hệ giữa
shaft power, RPM, pitch, advance ratio và aerodynamic torque. Khi bổ sung
engine/governor, cần mở rộng state/input một cách tường minh thay vì sửa
`LoadAccumulator` hoặc `RigidBody6DOF`.

## 6. Math type dùng chung

`navion::propeller::Vec3` và `Mat3` riêng đã được loại bỏ. Propeller hiện dùng:

```cpp
navion::Vec3
navion::Matrix3
```

`Matrix3::transposed()` được bổ sung vào math core để tạo
`C_propeller<-body` từ `C_body<-propeller`. Việc dùng một type chung loại bỏ
adapter vector/matrix và tránh trộn frame ở biên component.

## 7. Đăng ký component

```cpp
#include "navion/components/propeller/PropellerModel.hpp"

navion::NavionModel model(massProperties);

model.addLoadComponent(
    std::make_unique<navion::propeller::PropellerComponent>(
        navion::propeller::makeEstimatedNavionNaca5868_9Parameters()
    )
);
```

Không gọi propeller trực tiếp trong vòng lặp RK4. `NavionModel` sẽ gọi component
cùng VS, HS và các component tương lai tại từng derivative evaluation.

## 8. Thay đổi tên dữ liệu Propeller

Các field được chuẩn hóa sang style và đơn vị của project chung. Một số ánh xạ
quan trọng:

| Propeller upload | Stage 3 |
| --- | --- |
| `radius_m` | `radiusM` |
| `rotationRate_rad_s` | `rotationRateRadps` |
| `rotatingInertia_kg_m2` | `rotatingInertiaKgM2` |
| `hubPositionFromCg_body_m` | `hubPositionFromCgBodyM` |
| `force_body_N` | `forceBodyN` |
| `totalMomentAtCg_body_Nm` | `totalMomentAtCgBodyNm` |
| `inducedVelocity_m_s` | `inducedVelocityMps` |
| `chord_m`, `pitch_rad` | `chordM`, `pitchRad` |

Factory `makeEstimatedNavionNaca5868_9Parameters()` đã được cập nhật tương ứng.

## 9. Giới hạn dữ liệu

Kiến trúc và kernel đã tích hợp, nhưng factory hiện vẫn chứa geometry,
Clark-Y polar, RPM, inertia và installation ước lượng từ Propeller V1. Không
dùng thrust/torque hiện tại làm dữ liệu validation Navion trước khi thay bằng
geometry/polar đã số hóa và dữ liệu vận hành đáng tin cậy.
