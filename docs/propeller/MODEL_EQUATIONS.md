# Propeller V1 - phương trình và quy ước triển khai

## 1. Phạm vi

Đây là mô hình propeller quasi-steady dành cho vòng lặp nonlinear 6-DOF. Tại
mỗi lần gọi, một biến ẩn đại số uniform induced inflow `lambda_i` được giải sao
cho thrust từ blade-element nhất quán với momentum theory. RPM và blade pitch
được giữ cố định.

Mô hình dùng SI và radian. Không có đại lượng độ hoặc đơn vị Anh trong runtime
API hay file CSV kết quả.

## 2. Hệ trục và chiều quay

- Body: `+x_b` về trước, `+y_b` sang phải, `+z_b` xuống dưới.
- Propeller: `+x_p` dọc shaft theo chiều thrust dương; disk là mặt phẳng
  `y_p-z_p`.
- `C_b_from_p` biến đổi thành phần vector từ propeller sang body.
- `rotationSign = +1` là quay dương theo quy tắc bàn tay phải quanh `+x_p`.
  Với shaft trùng body-x, phi công nhìn về mũi sẽ thấy quay thuận chiều kim đồng
  hồ.
- `psi=0` đặt blade theo `+y_p`.

Tại azimuth `psi`:

\[
\mathbf e_r^p=(0,\cos\psi,\sin\psi),\qquad
\mathbf e_t^p=s_\Omega\,\mathbf e_x^p\times\mathbf e_r^p.
\]

## 3. Động học hub và phần tử

Vận tốc hub tương đối với không khí:

\[
\mathbf V_h^b=\mathbf V_{CG/a}^b+
\boldsymbol\omega_{B/I}^b\times\mathbf r_{h/CG}^b,
\]

\[
\mathbf V_h^p=C_{p\leftarrow b}\mathbf V_h^b,\qquad
\boldsymbol\omega^p=C_{p\leftarrow b}\boldsymbol\omega^b.
\]

Các tham số vô thứ nguyên:

\[
\lambda_0=\frac{V_{h,x}^p}{\Omega R},\qquad
\mu_y=\frac{V_{h,y}^p}{\Omega R},\qquad
\mu_z=\frac{V_{h,z}^p}{\Omega R},
\]

\[
\mu=\sqrt{\mu_y^2+\mu_z^2},\qquad
J=\frac{V_{h,x}^p}{nD},\qquad n=\frac{\Omega}{2\pi}.
\]

Với một giá trị thử `lambda_i`, induced velocity là

\[
v_i=\lambda_i\Omega R.
\]

Vận tốc vật liệu của blade element tương đối với không khí:

\[
\mathbf U_e^p=\mathbf V_h^p+
\boldsymbol\omega^p\times\mathbf r_e^p+
\Omega r\mathbf e_t^p+v_i\mathbf e_x^p.
\]

Dấu `+v_i e_x` xuất hiện vì với thrust dương `+x_p`, propeller gia tốc không khí
về phía `-x_p`; vận tốc của blade tương đối với dòng induced vì vậy tăng theo
`+x_p`.

V1 bỏ thành phần spanwise và chỉ giữ

\[
U_a=\mathbf U_e^p\cdot\mathbf e_x^p,\qquad
U_t=\mathbf U_e^p\cdot\mathbf e_t^p.
\]

## 4. Blade-element phi tuyến

\[
W=\sqrt{U_a^2+U_t^2},\qquad
\phi=\operatorname{atan2}(U_a,U_t),\qquad
\alpha=\beta(r)-\phi.
\]

V1 nội suy một polar duy nhất:

\[
C_l=C_l(\alpha),\qquad C_d=C_d(\alpha).
\]

Tải trên một đơn vị span:

\[
L'=\frac12\rho W^2c(r)C_l,\qquad
D'=\frac12\rho W^2c(r)C_d.
\]

Không dùng xấp xỉ góc inflow nhỏ. Phép phân giải chính xác là

\[
F'_x=L'\cos\phi-D'\sin\phi,
\]

\[
F'_t=-L'\sin\phi-D'\cos\phi,
\]

\[
d\mathbf F^p=(F'_x\mathbf e_x^p+F'_t\mathbf e_t^p)dr,
\qquad
d\mathbf M_h^p=\mathbf r_e^p\times d\mathbf F^p.
\]

## 5. Tích phân disk-mean

Với `B` blades:

\[
\mathbf F_h^p=\frac{B}{2\pi}
\int_0^{2\pi}\int_{r_0}^{R}d\mathbf F^p\,d\psi,
\]

\[
\mathbf M_{h,aero}^p=\frac{B}{2\pi}
\int_0^{2\pi}\int_{r_0}^{R}
\mathbf r_e^p\times d\mathbf F^p\,d\psi.
\]

Code dùng midpoint quadrature theo cả `r` và `psi`. Moment dọc shaft trong
`M_h,aero` chính là aerodynamic reaction torque. Không được cộng thêm một
reaction-torque độc lập lần thứ hai. Hai moment vuông góc shaft là hub bending
moments, gồm mean P-factor và aerodynamic rate damping.

## 6. Uniform-inflow closure

Chuẩn hóa rotor dùng trong residual:

\[
C_{T,r}^{BE}=\frac{F_{h,x}^p}
{\rho\pi R^2(\Omega R)^2}.
\]

Momentum theory tổng quát cho uniform induced velocity dọc shaft:

\[
C_{T,r}^{MT}=2\lambda_i
\sqrt{\mu^2+(\lambda_0+\lambda_i)^2}.
\]

Residual cần giải:

\[
R_\lambda(\lambda_i)=C_{T,r}^{BE}(\lambda_i)-
C_{T,r}^{MT}(\lambda_i)=0.
\]

Mỗi lần residual được đánh giá, toàn bộ lưới blade-element được tính lại. Solver
dùng Newton với đạo hàm sai phân hữu hạn, nhưng mọi bước Newton ra ngoài
bracket đều được thay bằng bisection. Nếu hai endpoint không bracket nghiệm,
solver quét miền `lambda_i` đã cấu hình trước khi báo lỗi.

Ở static axial condition:

\[
\lambda_0=\mu=0\quad\Rightarrow\quad C_{T,r}=2\lambda_i^2.
\]

Chuẩn hóa propeller chỉ dùng cho diagnostic/validation:

\[
C_{T,p}=\frac{T}{\rho n^2D^4},\qquad
C_{Q,p}=\frac{Q}{\rho n^2D^5}.
\]

## 7. Tải đưa sang rigid body

\[
\mathbf F_{prop}^b=C_{b\leftarrow p}\mathbf F_h^p,
\]

\[
\mathbf M_{arm}^b=\mathbf r_{h/CG}^b\times\mathbf F_{prop}^b.
\]

Angular momentum của rotating assembly:

\[
\mathbf H_{prop}^b=s_\Omega I_{spin}\Omega\mathbf e_s^b.
\]

Gyroscopic moment:

\[
\mathbf M_{gyro}^b=-\boldsymbol\omega^b\times\mathbf H_{prop}^b.
\]

Tổng moment quanh CG:

\[
\mathbf M_{prop,CG}^b=
C_{b\leftarrow p}\mathbf M_{h,aero}^p+
\mathbf M_{arm}^b+\mathbf M_{gyro}^b.
\]

`PropellerComponent::computeLoad()` đóng gói đúng hai đại lượng cuối cùng vào
contract chung:

```text
BodyLoad.forceBodyN          = F_prop_body
BodyLoad.momentAboutCgBodyNm = M_prop_CG_body
```

Vì moment đã gồm full aerodynamic hub moment, `r × F` và gyroscopic moment,
`LoadAccumulator` chỉ cộng nó một lần. Không được cộng riêng reaction torque,
hub bending moment, arm moment hoặc gyro moment ở `NavionModel` hay
`RigidBody6DOF`.

Stage 3 đưa `-omega × H_prop` vào `BodyLoad`. Do đó `MassProperties` không được
mô hình hóa lại cùng spin angular momentum trong một phương trình mở rộng;
nếu làm cả hai, gyroscopic effect sẽ bị đếm hai lần.

## 8. Nguồn phương trình

| Khối | Nguồn chuẩn hóa | Cách dùng trong code |
|---|---|---|
| Hub velocity | Stevens, Eq. 8.2-1 | Viết lại dạng vector và đổi sang propeller frame |
| Momentum closure | Stevens, Eq. 8.2-5 và 8.2-6 | Vô thứ nguyên hóa thành scalar residual |
| Blade-element dependence on inflow | Stevens, Eq. 8.2-7 đến 8.2-10 | Giữ kiến trúc nhưng bỏ giả thiết chord/twist hằng và góc nhỏ |
| P-factor/rate damping | Stevens, Eq. 8.2-20 đến 8.2-25 | Không dùng closed form; thu trực tiếp từ tích phân `r cross dF` |
| Gyroscopic moment | Stevens, Eq. 8.2-28 và 8.2-29 | Triển khai trực tiếp `-omega cross H` |
| Hidden-state software flow | `MODEL_6DOF_CALCULATION_FLOW.md` và uniform inflow của UH-1 | BEMT nằm bên trong mỗi residual evaluation; tính tải cuối tại nghiệm hội tụ |
| Phân giải lift/drag chính xác | Hình học blade-element tổng quát | Phần mở rộng, không copy xấp xỉ `U_a/U_t` của UH-1 |

Nguồn Stevens được đối chiếu từ *Aircraft Control and Simulation*, 3rd ed.,
Section 8.2, trang in 631-639. Mã UH-1 được dùng làm tham khảo kiến trúc, không
được xem là nguồn duy nhất cho các phương trình khí động.

## 9. Giới hạn cố ý của V1

- uniform, quasi-steady axial induction;
- không có tangential induction/wake swirl;
- không có Prandtl tip/root loss;
- polar không phụ thuộc Reynolds hoặc Mach;
- không có compressibility correction dù có báo section Mach;
- không có propwash lên wing/fuselage/tail;
- không có ground effect hoặc nacelle/spinner interference;
- không có governor, engine/shaft dynamics hoặc giới hạn công suất hồi tiếp RPM;
- không có blade flapping, lead-lag, torsion hoặc aeroelasticity;
- không có phase-resolved blade-passage loads;
- không mô hình hóa đúng reverse flow, windmilling, feather hoặc reverse pitch.
