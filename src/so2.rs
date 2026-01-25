extern crate nalgebra as na;
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub};

/// SO2 implementation
#[derive(Debug, Clone, Copy)]
pub struct SO2<T: na::Scalar + na::ComplexField + na::RealField + Copy> {
    arr: na::Unit<na::Vector2<T>>, // w, x
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Default for SO2<T> {
    fn default() -> Self {
        Self {
            arr: na::Unit::new_unchecked(na::Vector2::new(na::convert(1.0), na::convert(0.0))),
        }
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> SO2<T> {
    pub fn new(q: na::Unit<na::Vector2<T>>) -> SO2<T> {
        Self { arr: q }
    }
    pub fn random() -> SO2<T>
    where
        rand::distributions::Standard: rand::distributions::Distribution<T>,
    {
        SO2 {
            arr: na::Unit::new_normalize(na::Vector2::new_random()),
        }
    }
    pub fn identity() -> SO2<T> {
        SO2 {
            arr: na::Unit::new_unchecked(na::Vector2::new(na::convert(1.0), na::convert(0.0))),
        }
    }

    pub fn from_angle(angle: &T) -> SO2<T> {
        debug_assert!(
            angle.is_finite(),
            "SO2::from_angle received non-finite angle"
        );
        let c: T = angle.cos();
        let s: T = angle.sin();
        debug_assert!(
            c.is_finite() && s.is_finite(),
            "SO2::from_angle produced non-finite cos/sin values"
        );
        SO2 {
            arr: na::Unit::new_unchecked(na::Vector2::new(na::convert(c), na::convert(s))),
        }
    }
    pub fn from_rot_mat(m: &na::Matrix2<T>) -> SO2<T> {
        #[cfg(debug_assertions)]
        {
            let det = m[(0, 0)] * m[(1, 1)] - m[(0, 1)] * m[(1, 0)];
            let one: T = na::convert(1.0);
            let epsilon: T = na::convert(1e-6);
            debug_assert!(
                (det - one).abs() < epsilon,
                "from_rot_mat: Matrix determinant is not 1.0 (got {:?}), may not be a valid rotation matrix",
                det
            );
        }
        SO2 {
            arr: na::Unit::new_normalize(na::Vector2::new(m[(0, 0)], m[(1, 0)])),
        }
    }
    pub fn from_two_unit_vectors(
        u: na::Unit<na::Vector2<T>>,
        v: na::Unit<na::Vector2<T>>,
    ) -> SO2<T> {
        let mut q: SO2<T> = SO2::identity();
        let d: T = u[0] * v[0] + u[1] * v[1];
        if d < na::convert(0.99999999) && d > na::convert(-0.99999999) {
            q = SO2 {
                arr: na::Unit::new_unchecked(na::Vector2::new(d, u[0] * v[1] - u[1] * v[0])),
            };
        } else if d < na::convert(-0.99999999) {
            q = SO2 {
                arr: na::Unit::new_unchecked(na::Vector2::new(na::convert(-1.0), na::convert(0.0))),
            };
        }
        q
    }
    pub fn from_complex(qw: T, qx: T) -> SO2<T> {
        SO2 {
            arr: na::Unit::new_normalize(na::Vector2::new(qw, qx)),
        }
    }

    pub fn from_complex_vec(qvec: &na::Vector2<T>) -> SO2<T> {
        SO2 {
            arr: na::Unit::new_normalize(*qvec),
        }
    }
    pub fn w(&self) -> T {
        self.arr[(0, 0)]
    }
    pub fn x(&self) -> T {
        self.arr[(1, 0)]
    }
    pub fn array(&self) -> na::Vector2<T> {
        self.arr.into_inner()
    }

    pub fn elements(&self) -> na::Vector2<T> {
        self.arr.into_inner()
    }

    pub fn data(&self) -> &[T] {
        self.arr.as_ref().as_slice()
    }

    pub fn copy(&self) -> SO2<T> {
        SO2 { arr: self.arr }
    }

    pub fn normalize(&mut self) {
        let normalized = na::Unit::new_normalize(self.arr.into_inner());
        self.arr = normalized;
    }

    pub fn normalized(&self) -> SO2<T> {
        let mut copy = self.copy();
        copy.normalize();
        copy
    }
    pub fn rotation_matrix(&self) -> na::Matrix2<T> {
        let c: T = self.w();
        let s: T = self.x();
        na::Matrix2::new(c, -s, s, c)
    }

    #[deprecated(
        since = "0.1.0",
        note = "Use rotation_matrix() instead to follow Rust naming conventions"
    )]
    pub fn R(&self) -> na::Matrix2<T> {
        self.rotation_matrix()
    }
    pub fn inverse(&self) -> SO2<T> {
        SO2::from_complex(self.w(), -self.x())
    }

    pub fn invert(&mut self) -> &mut Self {
        let new_x = -self.x();
        self.arr = na::Unit::new_unchecked(na::Vector2::new(self.w(), new_x));
        self
    }
    pub fn angle(&self) -> T {
        self.x().atan2(self.w())
    }

    pub fn otimes(&self, q: &SO2<T>) -> SO2<T> {
        SO2 {
            arr: na::Unit::new_normalize(na::Vector2::new(
                self.w() * q.w() - self.x() * q.x(),
                self.w() * q.x() + self.x() * q.w(),
            )),
        }
    }

    pub fn oplus(&self, delta: &na::Vector1<T>) -> SO2<T> {
        self.otimes(&SO2::exp_map(delta))
    }

    pub fn ominus(&self, q: &SO2<T>) -> na::Vector1<T> {
        let dq = q.inverse().otimes(self);
        SO2::log_map(&dq)
    }

    pub fn hat(omega: &na::Vector1<T>) -> na::Matrix2<T> {
        let zero: T = na::convert(0.0);
        na::Matrix2::new(zero, -omega[0], omega[0], zero)
    }

    pub fn vee(omega_mat: &na::Matrix2<T>) -> na::Vector1<T> {
        na::Vector1::new(omega_mat[(1, 0)])
    }

    pub fn log(q: &SO2<T>) -> na::Matrix2<T> {
        SO2::hat(&SO2::log_map(q))
    }

    pub fn log_map(q: &SO2<T>) -> na::Vector1<T> {
        na::Vector1::new(q.angle())
    }

    #[deprecated(
        since = "0.1.0",
        note = "Use log_map() instead to follow Rust naming conventions"
    )]
    pub fn Log(q: &SO2<T>) -> na::Vector1<T> {
        SO2::log_map(q)
    }

    pub fn exp(omega_mat: &na::Matrix2<T>) -> SO2<T> {
        SO2::exp_map(&SO2::vee(omega_mat))
    }

    pub fn exp_map(omega: &na::Vector1<T>) -> SO2<T> {
        SO2::from_angle(&omega[0])
    }

    #[deprecated(
        since = "0.1.0",
        note = "Use exp_map() instead to follow Rust naming conventions"
    )]
    pub fn Exp(omega: &na::Vector1<T>) -> SO2<T> {
        SO2::exp_map(omega)
    }

    pub fn cast<T2: na::Scalar + na::ComplexField + na::RealField + Copy>(&self) -> SO2<T2>
    where
        T: Into<T2>,
    {
        SO2 {
            arr: na::Unit::new_unchecked(na::Vector2::new(
                na::convert(self.w().into()),
                na::convert(self.x().into()),
            )),
        }
    }
}

// Operator implementations

// Mul for SO2 * SO2
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<SO2<T>> for SO2<T> {
    type Output = SO2<T>;
    fn mul(self, rhs: SO2<T>) -> SO2<T> {
        self.otimes(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&SO2<T>> for SO2<T> {
    type Output = SO2<T>;
    fn mul(self, rhs: &SO2<T>) -> SO2<T> {
        self.otimes(rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<SO2<T>> for &SO2<T> {
    type Output = SO2<T>;
    fn mul(self, rhs: SO2<T>) -> SO2<T> {
        self.otimes(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&SO2<T>> for &SO2<T> {
    type Output = SO2<T>;
    fn mul(self, rhs: &SO2<T>) -> SO2<T> {
        self.otimes(rhs)
    }
}

// MulAssign for SO2 *= SO2
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> MulAssign<SO2<T>> for SO2<T> {
    fn mul_assign(&mut self, rhs: SO2<T>) {
        *self = self.otimes(&rhs);
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> MulAssign<&SO2<T>> for SO2<T> {
    fn mul_assign(&mut self, rhs: &SO2<T>) {
        *self = self.otimes(rhs);
    }
}

// Mul for SO2 * f64 (scalar scaling)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<f64> for SO2<T> {
    type Output = SO2<T>;
    fn mul(self, s: f64) -> SO2<T> {
        let log_val = SO2::log_map(&self);
        let scaled = log_val * na::convert::<f64, T>(s);
        SO2::exp_map(&scaled)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<f64> for &SO2<T> {
    type Output = SO2<T>;
    fn mul(self, s: f64) -> SO2<T> {
        let log_val = SO2::log_map(self);
        let scaled = log_val * na::convert::<f64, T>(s);
        SO2::exp_map(&scaled)
    }
}

// f64 * SO2 (scalar scaling from left)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<SO2<T>> for f64 {
    type Output = SO2<T>;
    fn mul(self, q: SO2<T>) -> SO2<T> {
        let log_val = SO2::log_map(&q);
        let scaled = log_val * na::convert::<f64, T>(self);
        SO2::exp_map(&scaled)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&SO2<T>> for f64 {
    type Output = SO2<T>;
    fn mul(self, q: &SO2<T>) -> SO2<T> {
        let log_val = SO2::log_map(q);
        let scaled = log_val * na::convert::<f64, T>(self);
        SO2::exp_map(&scaled)
    }
}

// MulAssign for SO2 *= f64
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> MulAssign<f64> for SO2<T> {
    fn mul_assign(&mut self, s: f64) {
        let log_val = SO2::log_map(self);
        let scaled = log_val * na::convert::<f64, T>(s);
        *self = SO2::exp_map(&scaled);
    }
}

// Div for SO2 / f64
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Div<f64> for SO2<T> {
    type Output = SO2<T>;
    fn div(self, s: f64) -> SO2<T> {
        debug_assert!(
            s.abs() >= f64::EPSILON,
            "Division by zero in SO2 scalar division: {}",
            s
        );
        let log_val = SO2::log_map(&self);
        SO2::exp_map(&(log_val / na::convert::<f64, T>(s)))
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Div<f64> for &SO2<T> {
    type Output = SO2<T>;
    fn div(self, s: f64) -> SO2<T> {
        debug_assert!(
            s.abs() >= f64::EPSILON,
            "Division by zero in SO2 scalar division: {}",
            s
        );
        let log_val = SO2::log_map(self);
        SO2::exp_map(&(log_val / na::convert::<f64, T>(s)))
    }
}

// DivAssign for SO2 /= f64
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> DivAssign<f64> for SO2<T> {
    fn div_assign(&mut self, s: f64) {
        debug_assert!(
            s.abs() >= f64::EPSILON,
            "Division by zero in SO2 scalar division: {}",
            s
        );
        let log_val = SO2::log_map(self);
        *self = SO2::exp_map(&(log_val / na::convert::<f64, T>(s)));
    }
}

// Mul for SO2 * Vector2 (transform vector)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<na::Vector2<T>> for SO2<T> {
    type Output = na::Vector2<T>;
    fn mul(self, rhs: na::Vector2<T>) -> na::Vector2<T> {
        let self_x: T = self.x();
        let self_w: T = self.w();
        let v_x: T = rhs[0];
        let v_y: T = rhs[1];
        na::Vector2::new(self_w * v_x - self_x * v_y, self_w * v_y + self_x * v_x)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&na::Vector2<T>> for SO2<T> {
    type Output = na::Vector2<T>;
    fn mul(self, rhs: &na::Vector2<T>) -> na::Vector2<T> {
        let self_x: T = self.x();
        let self_w: T = self.w();
        let v_x: T = rhs[0];
        let v_y: T = rhs[1];
        na::Vector2::new(self_w * v_x - self_x * v_y, self_w * v_y + self_x * v_x)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<na::Vector2<T>> for &SO2<T> {
    type Output = na::Vector2<T>;
    fn mul(self, rhs: na::Vector2<T>) -> na::Vector2<T> {
        let self_x: T = self.x();
        let self_w: T = self.w();
        let v_x: T = rhs[0];
        let v_y: T = rhs[1];
        na::Vector2::new(self_w * v_x - self_x * v_y, self_w * v_y + self_x * v_x)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<&na::Vector2<T>> for &SO2<T> {
    type Output = na::Vector2<T>;
    fn mul(self, rhs: &na::Vector2<T>) -> na::Vector2<T> {
        let self_x: T = self.x();
        let self_w: T = self.w();
        let v_x: T = rhs[0];
        let v_y: T = rhs[1];
        na::Vector2::new(self_w * v_x - self_x * v_y, self_w * v_y + self_x * v_x)
    }
}

// Add for SO2 + Vector1 (oplus)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Add<na::Vector1<T>> for SO2<T> {
    type Output = SO2<T>;
    fn add(self, rhs: na::Vector1<T>) -> SO2<T> {
        self.oplus(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Add<&na::Vector1<T>> for SO2<T> {
    type Output = SO2<T>;
    fn add(self, rhs: &na::Vector1<T>) -> SO2<T> {
        self.oplus(rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Add<na::Vector1<T>> for &SO2<T> {
    type Output = SO2<T>;
    fn add(self, rhs: na::Vector1<T>) -> SO2<T> {
        self.oplus(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Add<&na::Vector1<T>> for &SO2<T> {
    type Output = SO2<T>;
    fn add(self, rhs: &na::Vector1<T>) -> SO2<T> {
        self.oplus(rhs)
    }
}

// AddAssign for SO2 += Vector1
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> AddAssign<na::Vector1<T>> for SO2<T> {
    fn add_assign(&mut self, rhs: na::Vector1<T>) {
        *self = self.oplus(&rhs);
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> AddAssign<&na::Vector1<T>>
    for SO2<T>
{
    fn add_assign(&mut self, rhs: &na::Vector1<T>) {
        *self = self.oplus(rhs);
    }
}

// Sub for SO2 - SO2 (ominus)
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Sub<SO2<T>> for SO2<T> {
    type Output = na::Vector1<T>;
    fn sub(self, rhs: SO2<T>) -> na::Vector1<T> {
        self.ominus(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Sub<&SO2<T>> for SO2<T> {
    type Output = na::Vector1<T>;
    fn sub(self, rhs: &SO2<T>) -> na::Vector1<T> {
        self.ominus(rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Sub<SO2<T>> for &SO2<T> {
    type Output = na::Vector1<T>;
    fn sub(self, rhs: SO2<T>) -> na::Vector1<T> {
        self.ominus(&rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Sub<&SO2<T>> for &SO2<T> {
    type Output = na::Vector1<T>;
    fn sub(self, rhs: &SO2<T>) -> na::Vector1<T> {
        self.ominus(rhs)
    }
}

// Display trait for printing
impl<T: na::Scalar + na::ComplexField + na::RealField + Copy + fmt::Display> fmt::Display
    for SO2<T>
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "SO(2): [ {}, {}i ]", self.w(), self.x())
    }
}

// Unit Tests
#[cfg(test)]
mod test {
    use super::*;
    use crate::so3::SO3;
    use na::{Vector1, Vector2, Vector3};

    static EPSILON: f64 = 1e-8;

    #[test]
    fn test_action() {
        let num_tests = 1000;
        for _ in 0..num_tests {
            let so2: SO2<f64> = SO2::random();
            let v2: Vector2<f64> = Vector2::new_random();
            let mut v3: Vector3<f64> = Vector3::zeros();
            v3[0] = v2[0];
            v3[1] = v2[1];
            v3[2] = 0.0;
            let so3: SO3<f64> = SO3::from_euler(&0.0, &0.0, &so2.angle());

            assert!(so3.roll().abs() < EPSILON);
            assert!(so3.pitch().abs() < EPSILON);
            assert!((so3.yaw() - so2.angle()).abs() < EPSILON);

            let qv2: Vector2<f64> = so2 * v2;
            let qv3: Vector3<f64> = so3 * v3;

            assert!((qv2[0] - qv3[0]).abs() < EPSILON);
            assert!((qv2[1] - qv3[1]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_inversion_and_composition() {
        let num_tests = 1000;
        for _ in 0..num_tests {
            let q1: SO2<f64> = SO2::random();
            let q2: SO2<f64> = SO2::random();

            let q2inv = q2.inverse();
            let q1p = q1 * q2 * q2inv;

            assert!((q1.w() - q1p.w()).abs() < EPSILON);
            assert!((q1.x() - q1p.x()).abs() < EPSILON);
        }
    }

    #[test]
    fn test_angle_conversions() {
        let num_tests = 50;
        for _ in 0..num_tests {
            let mut angle: Vector1<f64> = Vector1::new_random();
            angle *= std::f64::consts::PI;
            let q = SO2::from_angle(&angle[0]);
            let q2 = SO2::from_angle(&q.angle());

            assert!((q.w() - q2.w()).abs() < EPSILON);
            assert!((q.x() - q2.x()).abs() < EPSILON);
        }
    }

    #[test]
    fn test_plus_minus() {
        let num_tests = 50;
        for _ in 0..num_tests {
            let q1: SO2<f64> = SO2::random();
            let q12: Vector1<f64> = Vector1::new_random();
            let q2 = q1 + q12;
            let q12p = q2 - q1;
            assert!((q12[0] - q12p[0]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_chart_maps() {
        let num_tests = 50;
        for _ in 0..num_tests {
            let q: SO2<f64> = SO2::random();
            let w: Vector1<f64> = Vector1::new_random();

            // Test log_map then exp_map
            let q_log = SO2::log_map(&q);
            let q2 = SO2::exp_map(&q_log);
            assert!((q.w() - q2.w()).abs() < EPSILON);
            assert!((q.x() - q2.x()).abs() < EPSILON);

            // Test exp_map then log_map
            let w_exp = SO2::exp_map(&w);
            let w2 = SO2::log_map(&w_exp);
            assert!((w[0] - w2[0]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_constructors() {
        let q_vec = Vector2::new(1.0, 0.0);
        let q = SO2::from_complex_vec(&q_vec);
        assert!((q.w() - 1.0_f64).abs() < EPSILON);
        assert!((q.x() - 0.0_f64).abs() < EPSILON);

        // Test from two unit vectors
        let mut v1: Vector2<f64> = Vector2::new_random();
        v1 /= v1.norm();
        let mut v2: Vector2<f64> = Vector2::new_random();
        v2 /= v2.norm();
        let qv =
            SO2::from_two_unit_vectors(na::Unit::new_unchecked(v1), na::Unit::new_unchecked(v2));
        let qv2 =
            SO2::from_two_unit_vectors(na::Unit::new_unchecked(v2), na::Unit::new_unchecked(v1))
                .inverse();
        assert!((qv.w() - qv2.w()).abs() < EPSILON);
        assert!((qv.x() - qv2.x()).abs() < EPSILON);

        // Test specific angles
        let thpi2 = SO2::from_two_unit_vectors(
            na::Unit::new_unchecked(Vector2::new(0.0, 1.0)),
            na::Unit::new_unchecked(Vector2::new(1.0, 0.0)),
        )
        .angle();
        assert!((thpi2 - (-std::f64::consts::PI / 2.0)).abs() < EPSILON);

        let thpi2i = SO2::from_two_unit_vectors(
            na::Unit::new_unchecked(Vector2::new(1.0, 0.0)),
            na::Unit::new_unchecked(Vector2::new(0.0, 1.0)),
        )
        .angle();
        assert!((thpi2i - std::f64::consts::PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_mutable_array() {
        let q = SO2::<f64>::identity();
        let mut q_arr = q.array();
        q_arr[0] = 2.0;
        assert!((q_arr[0] - 2.0).abs() < EPSILON);
        assert!((q.w() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_scaling() {
        let qi = SO2::<f64>::identity();
        let qis = 5.0 * qi;
        assert!((qis.w() - qi.w()).abs() < EPSILON);
        assert!((qis.x() - qi.x()).abs() < EPSILON);

        let qr = SO2::<f64>::random();
        let qr2 = qr * 0.2; // if scale is too big, then the rotation will
                            // wrap around the circle, resulting in a reversed
                            // or truncated tangent vector which can't be inverted
                            // through scalar division
        let qr3 = qr2 / 0.2;
        assert!((qr.w() - qr3.w()).abs() < EPSILON);
        assert!((qr.x() - qr3.x()).abs() < EPSILON);
    }

    #[test]
    fn test_identity() {
        let q = SO2::<f64>::identity();
        assert!((q.w() - 1.0).abs() < EPSILON);
        assert!(q.x().abs() < EPSILON);
    }

    #[test]
    fn test_hat_vee() {
        let omega = Vector1::new(0.5);
        let omega_hat = SO2::<f64>::hat(&omega);
        let omega_vee = SO2::<f64>::vee(&omega_hat);
        assert!((omega[0] - omega_vee[0]).abs() < EPSILON);
    }

    #[test]
    fn test_log_exp_consistency() {
        let q = SO2::<f64>::random();
        let omega_mat = SO2::log(&q);
        let q2 = SO2::exp(&omega_mat);
        assert!((q.w() - q2.w()).abs() < EPSILON);
        assert!((q.x() - q2.x()).abs() < EPSILON);
    }

    #[test]
    fn test_normalize() {
        let mut q = SO2::from_complex(3.0, 4.0);
        q.normalize();
        let norm_sq: f64 = q.w() * q.w() + q.x() * q.x();
        let norm = norm_sq.sqrt();
        assert!((norm - 1.0_f64).abs() < EPSILON);
    }

    #[test]
    fn test_rotation_matrix() {
        let angle = std::f64::consts::PI / 4.0;
        let q = SO2::from_angle(&angle);
        let r = q.rotation_matrix();

        // Check that R is a valid rotation matrix
        let det = r[(0, 0)] * r[(1, 1)] - r[(0, 1)] * r[(1, 0)];
        assert!((det - 1.0).abs() < EPSILON);

        // Check orthogonality
        let rt_r = r.transpose() * r;
        assert!((rt_r[(0, 0)] - 1.0).abs() < EPSILON);
        assert!((rt_r[(1, 1)] - 1.0).abs() < EPSILON);
        assert!(rt_r[(0, 1)].abs() < EPSILON);
        assert!(rt_r[(1, 0)].abs() < EPSILON);
    }

    #[test]
    fn test_from_rotation_matrix() {
        let angle = std::f64::consts::PI / 3.0;
        let q = SO2::from_angle(&angle);
        let r = q.rotation_matrix();
        let q2 = SO2::from_rot_mat(&r);

        assert!((q.w() - q2.w()).abs() < EPSILON);
        assert!((q.x() - q2.x()).abs() < EPSILON);
    }

    #[test]
    fn test_display() {
        let q = SO2::<f64>::identity();
        let s = format!("{}", q);
        assert!(s.contains("SO(2)"));
    }
}
