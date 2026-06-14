extern crate nalgebra as na;
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub};

/// SO3 implementation
#[derive(Debug, Clone, Copy)]
pub struct SO3<T: na::Scalar + na::ComplexField + na::RealField + Copy> {
    arr: na::Unit<na::Vector4<T>>, // w, x, y, z
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Default for SO3<T> {
    fn default() -> Self {
        Self {
            arr: na::Unit::new_unchecked(na::Vector4::new(
                na::convert(1.0),
                na::convert(0.0),
                na::convert(0.0),
                na::convert(0.0),
            )),
        }
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> SO3<T> {
    pub fn new(q: na::Unit<na::Vector4<T>>) -> SO3<T> {
        Self { arr: q }
    }

    pub fn random() -> SO3<T>
    where
        rand::distributions::Standard: rand::distributions::Distribution<T>,
    {
        let mut q = SO3 {
            arr: na::Unit::new_normalize(na::Vector4::new_random()),
        };
        q.normalize();
        q
    }

    pub fn identity() -> SO3<T> {
        SO3 {
            arr: na::Unit::new_unchecked(na::Vector4::new(
                na::convert(1.0),
                na::convert(0.0),
                na::convert(0.0),
                na::convert(0.0),
            )),
        }
    }

    pub fn nans() -> SO3<T> {
        SO3 {
            arr: na::Unit::new_unchecked(na::Vector4::new(
                na::convert(f64::NAN),
                na::convert(f64::NAN),
                na::convert(f64::NAN),
                na::convert(f64::NAN),
            )),
        }
    }

    pub fn from_axis_angle(axis: &na::Vector3<T>, angle: &T) -> SO3<T> {
        let th2: T = *angle / na::convert(2.0);
        let axis_normalized = axis.normalize();
        let scale: T = th2.sin();
        let mut q = SO3 {
            arr: na::Unit::new_normalize(na::Vector4::new(
                th2.cos(),
                scale * axis_normalized[0],
                scale * axis_normalized[1],
                scale * axis_normalized[2],
            )),
        };
        q.normalize();
        q
    }

    pub fn from_euler(roll: &T, pitch: &T, yaw: &T) -> SO3<T> {
        let q_roll: SO3<T> = SO3::from_axis_angle(&na::Vector3::x(), roll);
        let q_pitch: SO3<T> = SO3::from_axis_angle(&na::Vector3::y(), pitch);
        let q_yaw: SO3<T> = SO3::from_axis_angle(&na::Vector3::z(), yaw);
        q_yaw * q_pitch * q_roll
    }

    pub fn from_rot_mat(m: &na::Matrix3<T>) -> SO3<T> {
        let r11 = m[(0, 0)];
        let r12 = m[(0, 1)];
        let r13 = m[(0, 2)];
        let r21 = m[(1, 0)];
        let r22 = m[(1, 1)];
        let r23 = m[(1, 2)];
        let r31 = m[(2, 0)];
        let r32 = m[(2, 1)];
        let r33 = m[(2, 2)];

        let two: T = na::convert(2.0);
        let quarter: T = na::convert(0.25);
        let (qw, qx, qy, qz);

        if r11 + r22 + r33 > T::zero() {
            let s = two * (T::one() + r11 + r22 + r33).sqrt();
            qw = quarter * s;
            qx = (r32 - r23) / s;
            qy = (r13 - r31) / s;
            qz = (r21 - r12) / s;
        } else if r11 > r22 && r11 > r33 {
            let s = two * (T::one() + r11 - r22 - r33).sqrt();
            qw = (r32 - r23) / s;
            qx = quarter * s;
            qy = (r21 + r12) / s;
            qz = (r31 + r13) / s;
        } else if r22 > r33 {
            let s = two * (T::one() + r22 - r11 - r33).sqrt();
            qw = (r13 - r31) / s;
            qx = (r21 + r12) / s;
            qy = quarter * s;
            qz = (r32 + r23) / s;
        } else {
            let s = two * (T::one() + r33 - r11 - r22).sqrt();
            qw = (r21 - r12) / s;
            qx = (r31 + r13) / s;
            qy = (r32 + r23) / s;
            qz = quarter * s;
        }

        let mut q = SO3::from_quat(qw, qx, qy, qz);
        q.normalize();
        q
    }

    pub fn from_two_unit_vectors(
        u: na::Unit<na::Vector3<T>>,
        v: na::Unit<na::Vector3<T>>,
    ) -> SO3<T> {
        let d: T = u.dot(&v);
        if d < na::convert(0.99999999) && d > na::convert(-0.99999999) {
            let invs: T = T::one() / (na::convert::<f64, T>(2.0) * (T::one() + d)).sqrt();
            let xyz = u.cross(&(v.scale(invs)));
            let mut q = SO3::from_quat(
                na::convert::<f64, T>(0.5) / invs,
                xyz[0],
                xyz[1],
                xyz[2],
            );
            q.normalize();
            q
        } else if d < na::convert(-0.99999999) {
            SO3::from_quat(T::zero(), T::zero(), T::one(), T::zero())
        } else {
            SO3::identity()
        }
    }

    pub fn from_quat(qw: T, qx: T, qy: T, qz: T) -> SO3<T> {
        SO3 {
            arr: na::Unit::new_unchecked(na::Vector4::new(qw, qx, qy, qz)),
        }
    }

    pub fn from_quat_vec(qvec: &na::Vector4<T>) -> SO3<T> {
        SO3 {
            arr: na::Unit::new_unchecked(*qvec),
        }
    }

    pub fn w(&self) -> T {
        self.arr[(0, 0)]
    }
    pub fn x(&self) -> T {
        self.arr[(1, 0)]
    }
    pub fn y(&self) -> T {
        self.arr[(2, 0)]
    }
    pub fn z(&self) -> T {
        self.arr[(3, 0)]
    }

    pub fn elements(&self) -> na::Vector4<T> {
        self.arr.into_inner()
    }

    pub fn array(&self) -> na::Vector4<T> {
        self.arr.into_inner()
    }

    pub fn data(&self) -> &[T] {
        self.arr.as_ref().as_slice()
    }

    pub fn copy(&self) -> SO3<T> {
        SO3 { arr: self.arr }
    }

    pub fn normalize(&mut self) {
        let v = self.arr.into_inner();
        let norm = v.norm();
        let mut normalized = v / norm;
        if normalized[0] < T::zero() {
            normalized = -normalized;
        }
        self.arr = na::Unit::new_unchecked(normalized);
    }

    pub fn normalized(&self) -> SO3<T> {
        let mut copy = self.copy();
        copy.normalize();
        copy
    }

    pub fn rotation_matrix(&self) -> na::Matrix3<T> {
        let wx = self.w() * self.x();
        let wy = self.w() * self.y();
        let wz = self.w() * self.z();
        let xx = self.x() * self.x();
        let xy = self.x() * self.y();
        let xz = self.x() * self.z();
        let yy = self.y() * self.y();
        let yz = self.y() * self.z();
        let zz = self.z() * self.z();
        let two: T = na::convert(2.0);
        na::Matrix3::new(
            T::one() - two * (yy + zz),
            two * (xy - wz),
            two * (xz + wy),
            two * (xy + wz),
            T::one() - two * (xx + zz),
            two * (yz - wx),
            two * (xz - wy),
            two * (yz + wx),
            T::one() - two * (xx + yy),
        )
    }

    #[deprecated(
        since = "0.1.0",
        note = "Use rotation_matrix() instead to follow Rust naming conventions"
    )]
    pub fn R(&self) -> na::Matrix3<T> {
        self.rotation_matrix()
    }

    pub fn inverse(&self) -> SO3<T> {
        SO3::from_quat(self.w(), -self.x(), -self.y(), -self.z())
    }

    pub fn invert(&mut self) -> &mut Self {
        let v = self.arr.into_inner();
        self.arr = na::Unit::new_unchecked(na::Vector4::new(v[0], -v[1], -v[2], -v[3]));
        self
    }

    pub fn roll(&self) -> T {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();
        ((w * x + y * z) * na::convert(2.0))
            .atan2(T::one() - (x * x + y * y) * na::convert(2.0))
    }

    pub fn pitch(&self) -> T {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();
        let val: T = (w * y - x * z) * na::convert(2.0);
        if val.abs() > T::one() {
            T::one().copysign(val) * na::convert(std::f64::consts::PI / 2.0)
        } else {
            val.asin()
        }
    }

    pub fn yaw(&self) -> T {
        let x = self.x();
        let y = self.y();
        let z = self.z();
        let w = self.w();
        ((w * z + x * y) * na::convert(2.0))
            .atan2(T::one() - (y * y + z * z) * na::convert(2.0))
    }

    pub fn to_euler(&self) -> na::Vector3<T> {
        na::Vector3::new(self.roll(), self.pitch(), self.yaw())
    }

    pub fn qmat_left(&self) -> na::Matrix4<T> {
        let w = self.w();
        let x = self.x();
        let y = self.y();
        let z = self.z();
        na::Matrix4::new(
            w, -x, -y, -z, x, w, -z, y, y, z, w, -x, z, -y, x, w,
        )
    }

    pub fn otimes(&self, q: &SO3<T>) -> SO3<T> {
        let sw = self.w();
        let sx = self.x();
        let sy = self.y();
        let sz = self.z();
        let qw = q.w();
        let qx = q.x();
        let qy = q.y();
        let qz = q.z();
        SO3 {
            arr: na::Unit::new_normalize(na::Vector4::new(
                sw * qw - sx * qx - sy * qy - sz * qz,
                sw * qx + sx * qw + sy * qz - sz * qy,
                sw * qy - sx * qz + sy * qw + sz * qx,
                sw * qz + sx * qy - sy * qx + sz * qw,
            )),
        }
    }

    pub fn oplus(&self, delta: &na::Vector3<T>) -> SO3<T> {
        self.otimes(&SO3::exp_map(delta))
    }

    pub fn ominus(&self, q: &SO3<T>) -> na::Vector3<T> {
        let mut dq = q.inverse().otimes(self);
        if dq.w() < T::zero() {
            let v = dq.arr.into_inner();
            dq.arr = na::Unit::new_unchecked(-v);
        }
        SO3::log_map(&dq)
    }

    pub fn hat(omega: &na::Vector3<T>) -> na::Matrix3<T> {
        let zero: T = T::zero();
        na::Matrix3::new(
            zero, -omega[2], omega[1], omega[2], zero, -omega[0], -omega[1], omega[0], zero,
        )
    }

    pub fn vee(omega_mat: &na::Matrix3<T>) -> na::Vector3<T> {
        na::Vector3::new(omega_mat[(2, 1)], omega_mat[(0, 2)], omega_mat[(1, 0)])
    }

    pub fn log(q: &SO3<T>) -> na::Matrix3<T> {
        SO3::hat(&SO3::log_map(q))
    }

    pub fn log_map(q: &SO3<T>) -> na::Vector3<T> {
        let qv = na::Vector3::new(q.x(), q.y(), q.z());
        let qw = q.w();
        let n = qv.norm();
        if n > na::convert(1e-4) {
            qv * (na::convert::<f64, T>(2.0) * n.atan2(qw) / n)
        } else {
            qv
        }
    }

    #[deprecated(
        since = "0.1.0",
        note = "Use log_map() instead to follow Rust naming conventions"
    )]
    pub fn Log(q: &SO3<T>) -> na::Vector3<T> {
        SO3::log_map(q)
    }

    pub fn exp(omega_mat: &na::Matrix3<T>) -> SO3<T> {
        SO3::exp_map(&SO3::vee(omega_mat))
    }

    pub fn exp_map(omega: &na::Vector3<T>) -> SO3<T> {
        let th = omega.norm();
        if th > na::convert(1e-4) {
            let u = omega / th;
            let half_th = th / na::convert(2.0);
            let s = half_th.sin();
            let mut q = SO3::from_quat(half_th.cos(), s * u[0], s * u[1], s * u[2]);
            q.normalize();
            q
        } else {
            let half: T = na::convert(0.5);
            let mut q = SO3::from_quat(
                T::one(),
                omega[0] * half,
                omega[1] * half,
                omega[2] * half,
            );
            q.normalize();
            q
        }
    }

    #[deprecated(
        since = "0.1.0",
        note = "Use exp_map() instead to follow Rust naming conventions"
    )]
    pub fn Exp(omega: &na::Vector3<T>) -> SO3<T> {
        SO3::exp_map(omega)
    }

    pub fn cast<T2: na::Scalar + na::ComplexField + na::RealField + Copy>(&self) -> SO3<T2>
    where
        T: Into<T2>,
    {
        SO3 {
            arr: na::Unit::new_unchecked(na::Vector4::new(
                na::convert(self.w().into()),
                na::convert(self.x().into()),
                na::convert(self.y().into()),
                na::convert(self.z().into()),
            )),
        }
    }
}

// Operator implementations

// Mul for SO3 * SO3
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<SO3<T>> for SO3<T> {
    type Output = SO3<T>;
    fn mul(self, rhs: SO3<T>) -> SO3<T> {
        self.otimes(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&SO3<T>> for SO3<T> {
    type Output = SO3<T>;
    fn mul(self, rhs: &SO3<T>) -> SO3<T> {
        self.otimes(rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<SO3<T>> for &SO3<T> {
    type Output = SO3<T>;
    fn mul(self, rhs: SO3<T>) -> SO3<T> {
        self.otimes(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&SO3<T>> for &SO3<T> {
    type Output = SO3<T>;
    fn mul(self, rhs: &SO3<T>) -> SO3<T> {
        self.otimes(rhs)
    }
}

// MulAssign for SO3 *= SO3
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> MulAssign<SO3<T>> for SO3<T> {
    fn mul_assign(&mut self, rhs: SO3<T>) {
        *self = self.otimes(&rhs);
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> MulAssign<&SO3<T>> for SO3<T> {
    fn mul_assign(&mut self, rhs: &SO3<T>) {
        *self = self.otimes(rhs);
    }
}

// Mul for SO3 * f64 (scalar scaling)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<f64> for SO3<T> {
    type Output = SO3<T>;
    fn mul(self, s: f64) -> SO3<T> {
        let log_val = SO3::log_map(&self);
        SO3::exp_map(&(log_val * na::convert::<f64, T>(s)))
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<f64> for &SO3<T> {
    type Output = SO3<T>;
    fn mul(self, s: f64) -> SO3<T> {
        let log_val = SO3::log_map(self);
        SO3::exp_map(&(log_val * na::convert::<f64, T>(s)))
    }
}

// f64 * SO3 (scalar scaling from left)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<SO3<T>> for f64 {
    type Output = SO3<T>;
    fn mul(self, q: SO3<T>) -> SO3<T> {
        let log_val = SO3::log_map(&q);
        SO3::exp_map(&(log_val * na::convert::<f64, T>(self)))
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&SO3<T>> for f64 {
    type Output = SO3<T>;
    fn mul(self, q: &SO3<T>) -> SO3<T> {
        let log_val = SO3::log_map(q);
        SO3::exp_map(&(log_val * na::convert::<f64, T>(self)))
    }
}

// MulAssign for SO3 *= f64
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> MulAssign<f64> for SO3<T> {
    fn mul_assign(&mut self, s: f64) {
        let log_val = SO3::log_map(self);
        *self = SO3::exp_map(&(log_val * na::convert::<f64, T>(s)));
    }
}

// Div for SO3 / f64
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Div<f64> for SO3<T> {
    type Output = SO3<T>;
    fn div(self, s: f64) -> SO3<T> {
        debug_assert!(
            s.abs() >= f64::EPSILON,
            "Division by zero in SO3 scalar division: {}",
            s
        );
        let log_val = SO3::log_map(&self);
        SO3::exp_map(&(log_val / na::convert::<f64, T>(s)))
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Div<f64> for &SO3<T> {
    type Output = SO3<T>;
    fn div(self, s: f64) -> SO3<T> {
        debug_assert!(
            s.abs() >= f64::EPSILON,
            "Division by zero in SO3 scalar division: {}",
            s
        );
        let log_val = SO3::log_map(self);
        SO3::exp_map(&(log_val / na::convert::<f64, T>(s)))
    }
}

// DivAssign for SO3 /= f64
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> DivAssign<f64> for SO3<T> {
    fn div_assign(&mut self, s: f64) {
        debug_assert!(
            s.abs() >= f64::EPSILON,
            "Division by zero in SO3 scalar division: {}",
            s
        );
        let log_val = SO3::log_map(self);
        *self = SO3::exp_map(&(log_val / na::convert::<f64, T>(s)));
    }
}

// Mul for SO3 * Vector3 (transform vector)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<na::Vector3<T>> for SO3<T> {
    type Output = na::Vector3<T>;
    fn mul(self, rhs: na::Vector3<T>) -> na::Vector3<T> {
        let qv = na::Vector3::new(self.x(), self.y(), self.z());
        let t = rhs.cross(&qv) * na::convert::<f64, T>(2.0);
        rhs - t * self.w() + t.cross(&qv)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&na::Vector3<T>> for SO3<T> {
    type Output = na::Vector3<T>;
    fn mul(self, rhs: &na::Vector3<T>) -> na::Vector3<T> {
        let qv = na::Vector3::new(self.x(), self.y(), self.z());
        let t = rhs.cross(&qv) * na::convert::<f64, T>(2.0);
        rhs - t * self.w() + t.cross(&qv)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<na::Vector3<T>> for &SO3<T> {
    type Output = na::Vector3<T>;
    fn mul(self, rhs: na::Vector3<T>) -> na::Vector3<T> {
        let qv = na::Vector3::new(self.x(), self.y(), self.z());
        let t = rhs.cross(&qv) * na::convert::<f64, T>(2.0);
        rhs - t * self.w() + t.cross(&qv)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&na::Vector3<T>> for &SO3<T> {
    type Output = na::Vector3<T>;
    fn mul(self, rhs: &na::Vector3<T>) -> na::Vector3<T> {
        let qv = na::Vector3::new(self.x(), self.y(), self.z());
        let t = rhs.cross(&qv) * na::convert::<f64, T>(2.0);
        rhs - t * self.w() + t.cross(&qv)
    }
}

// Add for SO3 + Vector3 (oplus)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Add<na::Vector3<T>> for SO3<T> {
    type Output = SO3<T>;
    fn add(self, rhs: na::Vector3<T>) -> SO3<T> {
        self.oplus(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Add<&na::Vector3<T>> for SO3<T> {
    type Output = SO3<T>;
    fn add(self, rhs: &na::Vector3<T>) -> SO3<T> {
        self.oplus(rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Add<na::Vector3<T>> for &SO3<T> {
    type Output = SO3<T>;
    fn add(self, rhs: na::Vector3<T>) -> SO3<T> {
        self.oplus(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Add<&na::Vector3<T>> for &SO3<T> {
    type Output = SO3<T>;
    fn add(self, rhs: &na::Vector3<T>) -> SO3<T> {
        self.oplus(rhs)
    }
}

// AddAssign for SO3 += Vector3
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> AddAssign<na::Vector3<T>> for SO3<T> {
    fn add_assign(&mut self, rhs: na::Vector3<T>) {
        *self = self.oplus(&rhs);
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> AddAssign<&na::Vector3<T>>
    for SO3<T>
{
    fn add_assign(&mut self, rhs: &na::Vector3<T>) {
        *self = self.oplus(rhs);
    }
}

// Sub for SO3 - SO3 (ominus)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Sub<SO3<T>> for SO3<T> {
    type Output = na::Vector3<T>;
    fn sub(self, rhs: SO3<T>) -> na::Vector3<T> {
        self.ominus(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Sub<&SO3<T>> for SO3<T> {
    type Output = na::Vector3<T>;
    fn sub(self, rhs: &SO3<T>) -> na::Vector3<T> {
        self.ominus(rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Sub<SO3<T>> for &SO3<T> {
    type Output = na::Vector3<T>;
    fn sub(self, rhs: SO3<T>) -> na::Vector3<T> {
        self.ominus(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Sub<&SO3<T>> for &SO3<T> {
    type Output = na::Vector3<T>;
    fn sub(self, rhs: &SO3<T>) -> na::Vector3<T> {
        self.ominus(rhs)
    }
}

// Display trait for printing
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy + fmt::Display> fmt::Display
    for SO3<T>
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "SO(3): [ {}, {}i, {}j, {}k ]",
            self.w(),
            self.x(),
            self.y(),
            self.z()
        )
    }
}

// Unit Tests
#[cfg(test)]
mod test {
    use super::*;
    use na::{Vector3, Vector4};

    static EPSILON: f64 = 1e-8;

    #[test]
    fn test_action() {
        let num_tests = 1000;
        for _ in 0..num_tests {
            let q: SO3<f64> = SO3::random();
            let v: Vector3<f64> = Vector3::new_random();

            let qv1 = q * v;
            let qv2 = q.rotation_matrix() * v;

            assert!((qv1[0] - qv2[0]).abs() < EPSILON);
            assert!((qv1[1] - qv2[1]).abs() < EPSILON);
            assert!((qv1[2] - qv2[2]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_inversion_and_composition() {
        let num_tests = 1000;
        for _ in 0..num_tests {
            let q1: SO3<f64> = SO3::random();
            let q2: SO3<f64> = SO3::random();

            let q2inv = q2.inverse();
            let q1p = q1 * q2 * q2inv;

            assert!((q1.w() - q1p.w()).abs() < EPSILON);
            assert!((q1.x() - q1p.x()).abs() < EPSILON);
            assert!((q1.y() - q1p.y()).abs() < EPSILON);
            assert!((q1.z() - q1p.z()).abs() < EPSILON);
        }
    }

    #[test]
    fn test_euler_conversions() {
        let num_tests = 50;
        for _ in 0..num_tests {
            let mut euler: Vector3<f64> = Vector3::new_random();
            euler *= std::f64::consts::PI;
            let q = SO3::from_euler(&euler[0], &euler[1], &euler[2]);
            let q2 = SO3::from_euler(&q.roll(), &q.pitch(), &q.yaw());

            assert!((q.w() - q2.w()).abs() < EPSILON);
            assert!((q.x() - q2.x()).abs() < EPSILON);
            assert!((q.y() - q2.y()).abs() < EPSILON);
            assert!((q.z() - q2.z()).abs() < EPSILON);
        }
    }

    #[test]
    fn test_plus_minus() {
        let num_tests = 50;
        for _ in 0..num_tests {
            let q1: SO3<f64> = SO3::random();
            let q12: Vector3<f64> = Vector3::new_random();
            let q2 = q1 + q12;
            let q12p = q2 - q1;
            assert!((q12[0] - q12p[0]).abs() < EPSILON);
            assert!((q12[1] - q12p[1]).abs() < EPSILON);
            assert!((q12[2] - q12p[2]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_chart_maps() {
        let num_tests = 50;
        for _ in 0..num_tests {
            let q: SO3<f64> = SO3::random();
            let w: Vector3<f64> = Vector3::new_random();

            let q_log = SO3::log_map(&q);
            let q2 = SO3::exp_map(&q_log);
            assert!((q.w() - q2.w()).abs() < EPSILON);
            assert!((q.x() - q2.x()).abs() < EPSILON);
            assert!((q.y() - q2.y()).abs() < EPSILON);
            assert!((q.z() - q2.z()).abs() < EPSILON);

            let w_exp = SO3::exp_map(&w);
            let w2 = SO3::log_map(&w_exp);
            assert!((w[0] - w2[0]).abs() < EPSILON);
            assert!((w[1] - w2[1]).abs() < EPSILON);
            assert!((w[2] - w2[2]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_constructors() {
        let q_vec = Vector4::new(1.0, 1.0, 0.0, 0.0_f64);
        let q = SO3::from_quat_vec(&q_vec);
        assert!((q.w() - 1.0_f64).abs() < EPSILON);
        assert!((q.x() - 1.0_f64).abs() < EPSILON);
        assert!((q.y() - 0.0_f64).abs() < EPSILON);
        assert!((q.z() - 0.0_f64).abs() < EPSILON);

        let mut v1: Vector3<f64> = Vector3::new_random();
        v1 /= v1.norm();
        let mut v2: Vector3<f64> = Vector3::new_random();
        v2 /= v2.norm();
        let qv = SO3::from_two_unit_vectors(
            na::Unit::new_unchecked(v1),
            na::Unit::new_unchecked(v2),
        );
        let qv2 = SO3::from_two_unit_vectors(
            na::Unit::new_unchecked(v2),
            na::Unit::new_unchecked(v1),
        )
        .inverse();
        assert!((qv.w() - qv2.w()).abs() < EPSILON);
        assert!((qv.x() - qv2.x()).abs() < EPSILON);
        assert!((qv.y() - qv2.y()).abs() < EPSILON);
        assert!((qv.z() - qv2.z()).abs() < EPSILON);
    }

    #[test]
    fn test_mutable_array() {
        let q = SO3::<f64>::identity();
        let mut q_arr = q.array();
        q_arr[0] = 2.0;
        assert!((q_arr[0] - 2.0).abs() < EPSILON);
        assert!((q.w() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_scaling() {
        let qi = SO3::<f64>::identity();
        let qis = 5.0 * qi;
        assert!((qis.w() - qi.w()).abs() < EPSILON);
        assert!((qis.x() - qi.x()).abs() < EPSILON);
        assert!((qis.y() - qi.y()).abs() < EPSILON);
        assert!((qis.z() - qi.z()).abs() < EPSILON);

        let qr = SO3::<f64>::random();
        let qr2 = qr * 0.2;
        let qr3 = qr2 / 0.2;
        assert!((qr.w() - qr3.w()).abs() < EPSILON);
        assert!((qr.x() - qr3.x()).abs() < EPSILON);
        assert!((qr.y() - qr3.y()).abs() < EPSILON);
        assert!((qr.z() - qr3.z()).abs() < EPSILON);
    }

    #[test]
    fn test_identity() {
        let q = SO3::<f64>::identity();
        assert!((q.w() - 1.0).abs() < EPSILON);
        assert!(q.x().abs() < EPSILON);
        assert!(q.y().abs() < EPSILON);
        assert!(q.z().abs() < EPSILON);
    }

    #[test]
    fn test_hat_vee() {
        let omega = Vector3::new(0.1, 0.2, 0.3_f64);
        let omega_hat = SO3::<f64>::hat(&omega);
        let omega_vee = SO3::<f64>::vee(&omega_hat);
        assert!((omega[0] - omega_vee[0]).abs() < EPSILON);
        assert!((omega[1] - omega_vee[1]).abs() < EPSILON);
        assert!((omega[2] - omega_vee[2]).abs() < EPSILON);
    }

    #[test]
    fn test_log_exp_consistency() {
        let q = SO3::<f64>::random();
        let omega_mat = SO3::log(&q);
        let q2 = SO3::exp(&omega_mat);
        assert!((q.w() - q2.w()).abs() < EPSILON);
        assert!((q.x() - q2.x()).abs() < EPSILON);
        assert!((q.y() - q2.y()).abs() < EPSILON);
        assert!((q.z() - q2.z()).abs() < EPSILON);
    }

    #[test]
    fn test_normalize() {
        let mut q = SO3::from_quat(1.0_f64, 2.0, 3.0, 4.0);
        q.normalize();
        let norm_sq: f64 = q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
        assert!((norm_sq.sqrt() - 1.0).abs() < EPSILON);
        assert!(q.w() >= 0.0);
    }

    #[test]
    fn test_rotation_matrix() {
        let q = SO3::from_euler(&0.1_f64, &0.2, &0.3);
        let r = q.rotation_matrix();

        // Check determinant is 1
        let det = r[(0, 0)] * (r[(1, 1)] * r[(2, 2)] - r[(1, 2)] * r[(2, 1)])
            - r[(0, 1)] * (r[(1, 0)] * r[(2, 2)] - r[(1, 2)] * r[(2, 0)])
            + r[(0, 2)] * (r[(1, 0)] * r[(2, 1)] - r[(1, 1)] * r[(2, 0)]);
        assert!((det - 1.0).abs() < EPSILON);

        // Check R^T R = I
        let rt_r = r.transpose() * r;
        assert!((rt_r[(0, 0)] - 1.0).abs() < EPSILON);
        assert!((rt_r[(1, 1)] - 1.0).abs() < EPSILON);
        assert!((rt_r[(2, 2)] - 1.0).abs() < EPSILON);
        assert!(rt_r[(0, 1)].abs() < EPSILON);
        assert!(rt_r[(0, 2)].abs() < EPSILON);
        assert!(rt_r[(1, 2)].abs() < EPSILON);
    }

    #[test]
    fn test_from_rotation_matrix() {
        let q = SO3::from_euler(&0.1_f64, &0.2, &0.3);
        let r = q.rotation_matrix();
        let q2 = SO3::from_rot_mat(&r);

        assert!((q.w() - q2.w()).abs() < EPSILON);
        assert!((q.x() - q2.x()).abs() < EPSILON);
        assert!((q.y() - q2.y()).abs() < EPSILON);
        assert!((q.z() - q2.z()).abs() < EPSILON);
    }

    #[test]
    fn test_display() {
        let q = SO3::<f64>::identity();
        let s = format!("{}", q);
        assert!(s.contains("SO(3)"));
    }

    #[test]
    fn test_to_euler() {
        let q = SO3::from_euler(&0.1_f64, &0.2, &0.3);
        let e = q.to_euler();
        assert!((e[0] - q.roll()).abs() < EPSILON);
        assert!((e[1] - q.pitch()).abs() < EPSILON);
        assert!((e[2] - q.yaw()).abs() < EPSILON);
    }

    #[test]
    fn test_qmat_left() {
        let q1: SO3<f64> = SO3::random();
        let q2: SO3<f64> = SO3::random();
        let q3 = q1.otimes(&q2);
        let q3_via_mat = q1.qmat_left() * q2.elements();
        assert!((q3.w() - q3_via_mat[0]).abs() < EPSILON);
        assert!((q3.x() - q3_via_mat[1]).abs() < EPSILON);
        assert!((q3.y() - q3_via_mat[2]).abs() < EPSILON);
        assert!((q3.z() - q3_via_mat[3]).abs() < EPSILON);
    }
}
