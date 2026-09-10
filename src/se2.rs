//! Two-dimensional rigid transforms.

use crate::na;
use crate::SO2;
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub};

/// A member of SE(2), stored as `[tx, ty, qw, qx]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SE2<T: na::RealField + Copy> {
    arr: na::Vector4<T>,
}

impl<T: na::RealField + Copy> Default for SE2<T> {
    fn default() -> Self {
        Self::identity()
    }
}

impl<T: na::RealField + Copy> SE2<T> {
    /// Constructs a transform from its scalar-first coefficient vector.
    pub fn new(arr: na::Vector4<T>) -> Self {
        Self { arr }
    }

    /// Constructs a transform from translation and rotation components.
    pub fn from_parts(translation: na::Vector2<T>, rotation: SO2<T>) -> Self {
        Self::from_coeffs(translation[0], translation[1], rotation.w(), rotation.x())
    }

    /// Constructs a transform from its individual coefficients.
    pub fn from_coeffs(tx: T, ty: T, qw: T, qx: T) -> Self {
        Self {
            arr: na::Vector4::new(tx, ty, qw, qx),
        }
    }

    /// Constructs a transform from a homogeneous matrix.
    pub fn from_homogeneous(matrix: &na::Matrix3<T>) -> Self {
        let rotation = SO2::from_rot_mat(&na::Matrix2::new(
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(1, 0)],
            matrix[(1, 1)],
        ));
        Self::from_parts(na::Vector2::new(matrix[(0, 2)], matrix[(1, 2)]), rotation)
    }

    /// Returns a random transform with a normalized rotation.
    pub fn random() -> Self
    where
        rand::distributions::Standard: rand::distributions::Distribution<T>,
    {
        Self::from_parts(na::Vector2::new_random(), SO2::random())
    }

    /// Returns the identity transform.
    pub fn identity() -> Self {
        Self::from_coeffs(T::zero(), T::zero(), T::one(), T::zero())
    }

    /// Returns a transform whose coefficients are all NaN.
    pub fn nans() -> Self {
        let nan = T::from_f64(f64::NAN).expect("real scalar must represent NaN");
        Self::from_coeffs(nan, nan, nan, nan)
    }

    /// Returns the translation component.
    pub fn translation(&self) -> na::Vector2<T> {
        na::Vector2::new(self.arr[0], self.arr[1])
    }

    /// Replaces the translation component.
    pub fn set_translation(&mut self, translation: na::Vector2<T>) {
        self.arr[0] = translation[0];
        self.arr[1] = translation[1];
    }

    /// Returns the rotation component.
    pub fn rotation(&self) -> SO2<T> {
        SO2::from_complex(self.arr[2], self.arr[3])
    }

    /// Replaces the rotation component.
    pub fn set_rotation(&mut self, rotation: SO2<T>) {
        self.arr[2] = rotation.w();
        self.arr[3] = rotation.x();
    }

    /// Returns a copy of the scalar-first coefficient vector.
    pub fn elements(&self) -> na::Vector4<T> {
        self.arr
    }

    /// Alias for [`SE2::elements`].
    pub fn array(&self) -> na::Vector4<T> {
        self.elements()
    }

    /// Returns the contiguous scalar-first coefficient buffer.
    pub fn data(&self) -> &[T] {
        self.arr.as_slice()
    }

    /// Returns the mutable contiguous scalar-first coefficient buffer.
    pub fn data_mut(&mut self) -> &mut [T] {
        self.arr.as_mut_slice()
    }

    /// Returns the homogeneous matrix representation.
    pub fn homogeneous_matrix(&self) -> na::Matrix3<T> {
        let r = self.rotation().rotation_matrix();
        let t = self.translation();
        na::Matrix3::new(
            r[(0, 0)],
            r[(0, 1)],
            t[0],
            r[(1, 0)],
            r[(1, 1)],
            t[1],
            T::zero(),
            T::zero(),
            T::one(),
        )
    }

    /// Returns the inverse transform.
    pub fn inverse(&self) -> Self {
        let q_inv = self.rotation().inverse();
        Self::from_parts(-(q_inv * self.translation()), q_inv)
    }

    /// Inverts this transform in place.
    pub fn invert(&mut self) -> &mut Self {
        *self = self.inverse();
        self
    }

    /// Composes this transform with `other`.
    pub fn otimes(&self, other: &Self) -> Self {
        let q = self.rotation();
        Self::from_parts(
            self.translation() + q * other.translation(),
            q * other.rotation(),
        )
    }

    /// Applies a right-hand tangent perturbation.
    pub fn oplus(&self, delta: &na::Vector3<T>) -> Self {
        self.otimes(&Self::exp_map(delta))
    }

    /// Computes `Log(other.inverse() * self)`.
    pub fn ominus(&self, other: &Self) -> na::Vector3<T> {
        Self::log_map(&other.inverse().otimes(self))
    }

    /// Applies the transform to a point.
    pub fn transform_point(&self, point: &na::Vector2<T>) -> na::Vector2<T> {
        self.rotation() * point + self.translation()
    }

    /// Maps a tangent vector to the Lie algebra.
    pub fn hat(omega: &na::Vector3<T>) -> na::Matrix3<T> {
        na::Matrix3::new(
            T::zero(),
            -omega[2],
            omega[0],
            omega[2],
            T::zero(),
            omega[1],
            T::zero(),
            T::zero(),
            T::zero(),
        )
    }

    /// Maps a Lie-algebra matrix to its tangent vector.
    pub fn vee(omega: &na::Matrix3<T>) -> na::Vector3<T> {
        na::Vector3::new(omega[(0, 2)], omega[(1, 2)], omega[(1, 0)])
    }

    /// Returns the matrix logarithm in se(2).
    pub fn log(transform: &Self) -> na::Matrix3<T> {
        Self::hat(&Self::log_map(transform))
    }

    /// Returns the tangent-vector logarithm.
    pub fn log_map(transform: &Self) -> na::Vector3<T> {
        let angle = transform.rotation().angle();
        let theta = angle.abs();
        let j_inv = if theta > na::convert(1e-4) {
            let a = theta.sin() / (T::one() - theta.cos());
            let half = theta / na::convert(2.0);
            let skew = na::Matrix2::new(T::zero(), -T::one(), T::one(), T::zero());
            (na::Matrix2::identity() * a - skew) * half
        } else {
            na::Matrix2::identity()
        };
        let rho = j_inv * transform.translation();
        na::Vector3::new(rho[0], rho[1], angle)
    }

    /// Returns the group exponential of an se(2) matrix.
    pub fn exp(omega: &na::Matrix3<T>) -> Self {
        Self::exp_map(&Self::vee(omega))
    }

    /// Returns the group exponential of a tangent vector.
    pub fn exp_map(omega: &na::Vector3<T>) -> Self {
        let rho = na::Vector2::new(omega[0], omega[1]);
        let angle = omega[2];
        let theta = angle.abs();
        let q = SO2::exp_map(&na::Vector1::new(angle));
        let j = if theta > na::convert(1e-4) {
            let a = theta.sin() / theta;
            let b = (T::one() - theta.cos()) / theta;
            let skew = na::Matrix2::new(T::zero(), -T::one(), T::one(), T::zero());
            na::Matrix2::identity() * a + skew * b
        } else {
            na::Matrix2::identity()
        };
        Self::from_parts(j * rho, q)
    }

    /// Casts coefficients to another real scalar type.
    pub fn cast<T2: na::RealField + Copy + num_traits::NumCast>(&self) -> SE2<T2>
    where
        T: num_traits::NumCast,
    {
        SE2::new(
            self.arr
                .map(|value| num_traits::cast(value).expect("numeric cast failed")),
        )
    }
}

impl<T: na::RealField + Copy> Index<usize> for SE2<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.arr[index]
    }
}

impl<T: na::RealField + Copy> IndexMut<usize> for SE2<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.arr[index]
    }
}

impl<T: na::RealField + Copy> Mul for SE2<T> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        self.otimes(&rhs)
    }
}

impl<T: na::RealField + Copy> MulAssign for SE2<T> {
    fn mul_assign(&mut self, rhs: Self) {
        *self = self.otimes(&rhs);
    }
}

impl<T: na::RealField + Copy> Mul<na::Vector2<T>> for SE2<T> {
    type Output = na::Vector2<T>;

    fn mul(self, rhs: na::Vector2<T>) -> Self::Output {
        self.transform_point(&rhs)
    }
}

impl<T: na::RealField + Copy> Add<na::Vector3<T>> for SE2<T> {
    type Output = Self;

    fn add(self, rhs: na::Vector3<T>) -> Self::Output {
        self.oplus(&rhs)
    }
}

impl<T: na::RealField + Copy> AddAssign<na::Vector3<T>> for SE2<T> {
    fn add_assign(&mut self, rhs: na::Vector3<T>) {
        *self = self.oplus(&rhs);
    }
}

impl<T: na::RealField + Copy> Sub for SE2<T> {
    type Output = na::Vector3<T>;

    fn sub(self, rhs: Self) -> Self::Output {
        self.ominus(&rhs)
    }
}

impl<T: na::RealField + Copy> Mul<f64> for SE2<T> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::exp_map(&(Self::log_map(&self) * na::convert::<f64, T>(rhs)))
    }
}

impl<T: na::RealField + Copy> Mul<SE2<T>> for f64 {
    type Output = SE2<T>;

    fn mul(self, rhs: SE2<T>) -> Self::Output {
        rhs * self
    }
}

impl<T: na::RealField + Copy> MulAssign<f64> for SE2<T> {
    fn mul_assign(&mut self, rhs: f64) {
        *self = *self * rhs;
    }
}

impl<T: na::RealField + Copy> Div<f64> for SE2<T> {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        assert!(rhs != 0.0, "cannot divide an SE2 transform by zero");
        Self::exp_map(&(Self::log_map(&self) / na::convert::<f64, T>(rhs)))
    }
}

impl<T: na::RealField + Copy> DivAssign<f64> for SE2<T> {
    fn div_assign(&mut self, rhs: f64) {
        *self = *self / rhs;
    }
}

impl<T: na::RealField + Copy + fmt::Display> fmt::Display for SE2<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let t = self.translation();
        let q = self.rotation();
        write!(
            f,
            "SE(2): [ {}i, {}j ] [ {}, {}i ]",
            t[0],
            t[1],
            q.w(),
            q.x()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    fn assert_transform_close(actual: SE2<f64>, expected: SE2<f64>) {
        assert!((actual.elements() - expected.elements()).norm() < EPS);
    }

    #[test]
    fn constructors_and_accessors() {
        let q = SO2::from_angle(&0.4);
        let mut x = SE2::from_parts(na::Vector2::new(1.0, -2.0), q);
        assert_eq!(x.translation(), na::Vector2::new(1.0, -2.0));
        assert_eq!(x.rotation().elements(), q.elements());
        x.set_translation(na::Vector2::new(3.0, 4.0));
        x.set_rotation(SO2::identity());
        x.data_mut()[0] = 5.0;
        x[1] = 6.0;
        assert_eq!(x.elements(), na::Vector4::new(5.0, 6.0, 1.0, 0.0));
        assert!(SE2::<f64>::nans().data().iter().all(|value| value.is_nan()));
    }

    #[test]
    fn homogeneous_round_trip() {
        let x = SE2::from_parts(na::Vector2::new(1.2, -3.4), SO2::from_angle(&-0.7));
        assert_transform_close(SE2::from_homogeneous(&x.homogeneous_matrix()), x);
    }

    #[test]
    fn inverse_and_composition() {
        for _ in 0..1000 {
            let x = SE2::<f64>::random();
            assert_transform_close(x * x.inverse(), SE2::identity());
            let mut inverse = x;
            inverse.invert();
            assert_transform_close(inverse, x.inverse());
        }
    }

    #[test]
    fn action_matches_homogeneous_matrix() {
        for _ in 0..1000 {
            let x = SE2::<f64>::random();
            let point = na::Vector2::<f64>::new_random();
            let homogeneous = x.homogeneous_matrix() * na::Vector3::new(point[0], point[1], 1.0);
            assert!((x * point - homogeneous.fixed_rows::<2>(0)).norm() < EPS);
        }
    }

    #[test]
    fn hat_and_vee_are_inverses() {
        let omega = na::Vector3::new(1.0, -2.0, 0.3);
        assert_eq!(SE2::vee(&SE2::hat(&omega)), omega);
    }

    #[test]
    fn chart_maps_round_trip_for_both_angle_signs() {
        for angle in [-2.0, -0.3, -1e-6, 0.0, 1e-6, 0.3, 2.0] {
            let omega = na::Vector3::new(0.7, -0.2, angle);
            let recovered = SE2::log_map(&SE2::exp_map(&omega));
            assert!((recovered - omega).norm() < EPS);
            assert_transform_close(
                SE2::exp(&SE2::log(&SE2::exp_map(&omega))),
                SE2::exp_map(&omega),
            );
        }
    }

    #[test]
    fn plus_minus_and_scaling_round_trip() {
        let x = SE2::from_parts(na::Vector2::new(1.0, 2.0), SO2::from_angle(&0.4));
        let delta = na::Vector3::new(0.2, -0.1, 0.3);
        assert!(((x + delta) - x - delta).norm() < EPS);
        assert_transform_close((x * 0.25) / 0.25, x);
    }

    #[test]
    #[should_panic(expected = "cannot divide an SE2 transform by zero")]
    fn division_by_zero_panics_in_all_builds() {
        let _ = SE2::<f64>::identity() / 0.0;
    }

    #[test]
    fn supports_f32_and_lossy_casts() {
        let x = SE2::<f64>::identity().cast::<f32>();
        assert_eq!(x, SE2::<f32>::identity());
        let _ = x * 0.5;
    }

    #[test]
    fn display_identifies_the_group() {
        assert!(SE2::<f64>::identity().to_string().contains("SE(2)"));
    }

    #[test]
    fn assignment_and_left_scalar_operators() {
        let delta = na::Vector3::new(0.1, -0.2, 0.3);
        let mut x = SE2::<f64>::identity();
        assert_eq!(x[0], 0.0);
        assert_eq!(x.array(), x.elements());
        x *= SE2::identity();
        x += delta;
        let expected = SE2::exp_map(&delta);
        assert_transform_close(x, expected);
        assert_transform_close(0.5 * x, x * 0.5);
        x *= 0.5;
        x /= 0.5;
        assert_transform_close(x, expected);
    }
}
