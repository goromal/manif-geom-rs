//! Three-dimensional rigid transforms.

use crate::na;
use crate::SO3;
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub};

/// A member of SE(3), stored as `[tx, ty, tz, qw, qx, qy, qz]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SE3<T: na::RealField + Copy> {
    arr: na::SVector<T, 7>,
}

impl<T: na::RealField + Copy> Default for SE3<T> {
    fn default() -> Self {
        Self::identity()
    }
}

impl<T: na::RealField + Copy> SE3<T> {
    /// Constructs a transform from its scalar-first coefficient vector.
    pub fn new(arr: na::SVector<T, 7>) -> Self {
        Self { arr }
    }

    /// Constructs a transform from translation and rotation components.
    pub fn from_parts(translation: na::Vector3<T>, rotation: SO3<T>) -> Self {
        Self::from_coeffs(
            translation[0],
            translation[1],
            translation[2],
            rotation.w(),
            rotation.x(),
            rotation.y(),
            rotation.z(),
        )
    }

    /// Constructs a transform from its individual coefficients.
    #[allow(clippy::too_many_arguments)]
    pub fn from_coeffs(tx: T, ty: T, tz: T, qw: T, qx: T, qy: T, qz: T) -> Self {
        Self {
            arr: na::SVector::<T, 7>::from_row_slice(&[tx, ty, tz, qw, qx, qy, qz]),
        }
    }

    /// Constructs a transform from translation and a nalgebra quaternion.
    pub fn from_quaternion(translation: na::Vector3<T>, rotation: na::Quaternion<T>) -> Self {
        Self::from_parts(translation, SO3::from_quaternion(rotation))
    }

    /// Constructs a transform from translation and a nalgebra unit quaternion.
    pub fn from_unit_quaternion(
        translation: na::Vector3<T>,
        rotation: na::UnitQuaternion<T>,
    ) -> Self {
        Self::from_parts(translation, SO3::from_unit_quaternion(rotation))
    }

    /// Constructs a transform from a homogeneous matrix.
    pub fn from_homogeneous(matrix: &na::Matrix4<T>) -> Self {
        let rotation = SO3::from_rot_mat(&na::Matrix3::new(
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(0, 2)],
            matrix[(1, 0)],
            matrix[(1, 1)],
            matrix[(1, 2)],
            matrix[(2, 0)],
            matrix[(2, 1)],
            matrix[(2, 2)],
        ));
        Self::from_parts(
            na::Vector3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]),
            rotation,
        )
    }

    /// Returns a random transform with a normalized rotation.
    pub fn random() -> Self
    where
        rand::distributions::Standard: rand::distributions::Distribution<T>,
    {
        Self::from_parts(na::Vector3::new_random(), SO3::random())
    }

    /// Returns the identity transform.
    pub fn identity() -> Self {
        Self::from_coeffs(
            T::zero(),
            T::zero(),
            T::zero(),
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
        )
    }

    /// Returns a transform whose coefficients are all NaN.
    pub fn nans() -> Self {
        let nan = T::from_f64(f64::NAN).expect("real scalar must represent NaN");
        Self::from_coeffs(nan, nan, nan, nan, nan, nan, nan)
    }

    /// Returns the translation component.
    pub fn translation(&self) -> na::Vector3<T> {
        na::Vector3::new(self.arr[0], self.arr[1], self.arr[2])
    }

    /// Replaces the translation component.
    pub fn set_translation(&mut self, translation: na::Vector3<T>) {
        self.arr[0] = translation[0];
        self.arr[1] = translation[1];
        self.arr[2] = translation[2];
    }

    /// Returns the rotation component.
    pub fn rotation(&self) -> SO3<T> {
        SO3::from_quat(self.arr[3], self.arr[4], self.arr[5], self.arr[6])
    }

    /// Replaces the rotation component.
    pub fn set_rotation(&mut self, rotation: SO3<T>) {
        self.arr[3] = rotation.w();
        self.arr[4] = rotation.x();
        self.arr[5] = rotation.y();
        self.arr[6] = rotation.z();
    }

    /// Returns a copy of the scalar-first coefficient vector.
    pub fn elements(&self) -> na::SVector<T, 7> {
        self.arr
    }

    /// Alias for [`SE3::elements`].
    pub fn array(&self) -> na::SVector<T, 7> {
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
    pub fn homogeneous_matrix(&self) -> na::Matrix4<T> {
        let r = self.rotation().rotation_matrix();
        let t = self.translation();
        na::Matrix4::new(
            r[(0, 0)],
            r[(0, 1)],
            r[(0, 2)],
            t[0],
            r[(1, 0)],
            r[(1, 1)],
            r[(1, 2)],
            t[1],
            r[(2, 0)],
            r[(2, 1)],
            r[(2, 2)],
            t[2],
            T::zero(),
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
    pub fn oplus(&self, delta: &na::Vector6<T>) -> Self {
        self.otimes(&Self::exp_map(delta))
    }

    /// Computes `Log(other.inverse() * self)`.
    pub fn ominus(&self, other: &Self) -> na::Vector6<T> {
        Self::log_map(&other.inverse().otimes(self))
    }

    /// Applies the transform to a point.
    pub fn transform_point(&self, point: &na::Vector3<T>) -> na::Vector3<T> {
        self.rotation() * point + self.translation()
    }

    /// Maps a tangent vector to the Lie algebra.
    pub fn hat(omega: &na::Vector6<T>) -> na::Matrix4<T> {
        let w_hat = SO3::hat(&na::Vector3::new(omega[3], omega[4], omega[5]));
        na::Matrix4::new(
            w_hat[(0, 0)],
            w_hat[(0, 1)],
            w_hat[(0, 2)],
            omega[0],
            w_hat[(1, 0)],
            w_hat[(1, 1)],
            w_hat[(1, 2)],
            omega[1],
            w_hat[(2, 0)],
            w_hat[(2, 1)],
            w_hat[(2, 2)],
            omega[2],
            T::zero(),
            T::zero(),
            T::zero(),
            T::zero(),
        )
    }

    /// Maps a Lie-algebra matrix to its tangent vector.
    pub fn vee(omega: &na::Matrix4<T>) -> na::Vector6<T> {
        let w = SO3::vee(&na::Matrix3::new(
            omega[(0, 0)],
            omega[(0, 1)],
            omega[(0, 2)],
            omega[(1, 0)],
            omega[(1, 1)],
            omega[(1, 2)],
            omega[(2, 0)],
            omega[(2, 1)],
            omega[(2, 2)],
        ));
        na::Vector6::new(
            omega[(0, 3)],
            omega[(1, 3)],
            omega[(2, 3)],
            w[0],
            w[1],
            w[2],
        )
    }

    /// Returns the matrix logarithm in se(3).
    pub fn log(transform: &Self) -> na::Matrix4<T> {
        Self::hat(&Self::log_map(transform))
    }

    /// Returns the tangent-vector logarithm.
    pub fn log_map(transform: &Self) -> na::Vector6<T> {
        let w = SO3::log_map(&transform.rotation());
        let theta = w.norm();
        let w_hat = SO3::hat(&w);
        let j_inv = if theta > na::convert(1e-4) {
            let a = theta.sin() / theta;
            let b = (T::one() - theta.cos()) / (theta * theta);
            let c = (T::one() - a) / (theta * theta);
            let e = (b - na::convert::<f64, T>(2.0) * c) / (na::convert::<f64, T>(2.0) * a);
            na::Matrix3::identity() - w_hat * na::convert::<f64, T>(0.5) + (w_hat * w_hat) * e
        } else {
            na::Matrix3::identity()
        };
        let rho = j_inv * transform.translation();
        na::Vector6::new(rho[0], rho[1], rho[2], w[0], w[1], w[2])
    }

    /// Returns the group exponential of an se(3) matrix.
    pub fn exp(omega: &na::Matrix4<T>) -> Self {
        Self::exp_map(&Self::vee(omega))
    }

    /// Returns the group exponential of a tangent vector.
    pub fn exp_map(omega: &na::Vector6<T>) -> Self {
        let rho = na::Vector3::new(omega[0], omega[1], omega[2]);
        let w = na::Vector3::new(omega[3], omega[4], omega[5]);
        let q = SO3::exp_map(&w);
        let theta = w.norm();
        let w_hat = SO3::hat(&w);
        let j = if theta > na::convert(1e-4) {
            let a = theta.sin() / theta;
            let b = (T::one() - theta.cos()) / (theta * theta);
            let c = (T::one() - a) / (theta * theta);
            na::Matrix3::identity() * a + w_hat * b + (w * w.transpose()) * c
        } else {
            na::Matrix3::identity()
        };
        Self::from_parts(j * rho, q)
    }

    /// Casts coefficients to another real scalar type.
    pub fn cast<T2: na::RealField + Copy + num_traits::NumCast>(&self) -> SE3<T2>
    where
        T: num_traits::NumCast,
    {
        SE3::new(
            self.arr
                .map(|value| num_traits::cast(value).expect("numeric cast failed")),
        )
    }
}

impl<T: na::RealField + Copy> Index<usize> for SE3<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.arr[index]
    }
}

impl<T: na::RealField + Copy> IndexMut<usize> for SE3<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.arr[index]
    }
}

impl<T: na::RealField + Copy> Mul for SE3<T> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        self.otimes(&rhs)
    }
}

impl<T: na::RealField + Copy> MulAssign for SE3<T> {
    fn mul_assign(&mut self, rhs: Self) {
        *self = self.otimes(&rhs);
    }
}

impl<T: na::RealField + Copy> Mul<na::Vector3<T>> for SE3<T> {
    type Output = na::Vector3<T>;

    fn mul(self, rhs: na::Vector3<T>) -> Self::Output {
        self.transform_point(&rhs)
    }
}

impl<T: na::RealField + Copy> Add<na::Vector6<T>> for SE3<T> {
    type Output = Self;

    fn add(self, rhs: na::Vector6<T>) -> Self::Output {
        self.oplus(&rhs)
    }
}

impl<T: na::RealField + Copy> AddAssign<na::Vector6<T>> for SE3<T> {
    fn add_assign(&mut self, rhs: na::Vector6<T>) {
        *self = self.oplus(&rhs);
    }
}

impl<T: na::RealField + Copy> Sub for SE3<T> {
    type Output = na::Vector6<T>;

    fn sub(self, rhs: Self) -> Self::Output {
        self.ominus(&rhs)
    }
}

impl<T: na::RealField + Copy> Mul<f64> for SE3<T> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::exp_map(&(Self::log_map(&self) * na::convert::<f64, T>(rhs)))
    }
}

impl<T: na::RealField + Copy> Mul<SE3<T>> for f64 {
    type Output = SE3<T>;

    fn mul(self, rhs: SE3<T>) -> Self::Output {
        rhs * self
    }
}

impl<T: na::RealField + Copy> MulAssign<f64> for SE3<T> {
    fn mul_assign(&mut self, rhs: f64) {
        *self = *self * rhs;
    }
}

impl<T: na::RealField + Copy> Div<f64> for SE3<T> {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        assert!(rhs != 0.0, "cannot divide an SE3 transform by zero");
        Self::exp_map(&(Self::log_map(&self) / na::convert::<f64, T>(rhs)))
    }
}

impl<T: na::RealField + Copy> DivAssign<f64> for SE3<T> {
    fn div_assign(&mut self, rhs: f64) {
        *self = *self / rhs;
    }
}

impl<T: na::RealField + Copy + fmt::Display> fmt::Display for SE3<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let t = self.translation();
        let q = self.rotation();
        write!(
            f,
            "SE(3): [ {}i, {}j, {}k ] [ {}, {}i, {}j, {}k ]",
            t[0],
            t[1],
            t[2],
            q.w(),
            q.x(),
            q.y(),
            q.z()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-8;

    fn assert_transform_close(actual: SE3<f64>, expected: SE3<f64>) {
        assert!((actual.homogeneous_matrix() - expected.homogeneous_matrix()).norm() < EPS);
    }

    #[test]
    fn constructors_and_accessors() {
        let q = SO3::from_euler(&0.1, &-0.2, &0.3);
        let mut x = SE3::from_parts(na::Vector3::new(1.0, -2.0, 3.0), q);
        assert_eq!(x.translation(), na::Vector3::new(1.0, -2.0, 3.0));
        assert_eq!(x.rotation().elements(), q.elements());
        x.set_translation(na::Vector3::new(4.0, 5.0, 6.0));
        x.set_rotation(SO3::identity());
        x.data_mut()[0] = 7.0;
        x[1] = 8.0;
        assert_eq!(
            x.elements(),
            na::SVector::<f64, 7>::from_row_slice(&[7.0, 8.0, 6.0, 1.0, 0.0, 0.0, 0.0])
        );
        assert!(SE3::<f64>::nans().data().iter().all(|value| value.is_nan()));
    }

    #[test]
    fn nalgebra_quaternion_constructors() {
        let q = na::UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3);
        let x = SE3::from_unit_quaternion(na::Vector3::new(1.0, 2.0, 3.0), q);
        assert!(
            (x.rotation().rotation_matrix() - q.to_rotation_matrix().into_inner()).norm() < EPS
        );
        let raw = *q.quaternion();
        let y = SE3::from_quaternion(x.translation(), raw);
        assert_transform_close(x, y);
    }

    #[test]
    fn homogeneous_round_trip() {
        let x = SE3::from_parts(
            na::Vector3::new(1.2, -3.4, 5.6),
            SO3::from_euler(&0.2, &-0.4, &0.7),
        );
        assert_transform_close(SE3::from_homogeneous(&x.homogeneous_matrix()), x);
    }

    #[test]
    fn inverse_and_composition() {
        for _ in 0..1000 {
            let x = SE3::<f64>::random();
            assert_transform_close(x * x.inverse(), SE3::identity());
            let mut inverse = x;
            inverse.invert();
            assert_transform_close(inverse, x.inverse());
        }
    }

    #[test]
    fn action_matches_homogeneous_matrix() {
        for _ in 0..1000 {
            let x = SE3::<f64>::random();
            let point = na::Vector3::<f64>::new_random();
            let homogeneous =
                x.homogeneous_matrix() * na::Vector4::new(point[0], point[1], point[2], 1.0);
            assert!((x * point - homogeneous.fixed_rows::<3>(0)).norm() < EPS);
        }
    }

    #[test]
    fn hat_and_vee_are_inverses() {
        let omega = na::Vector6::new(1.0, -2.0, 3.0, 0.1, -0.2, 0.3);
        assert_eq!(SE3::vee(&SE3::hat(&omega)), omega);
    }

    #[test]
    fn chart_maps_round_trip_across_angle_regimes() {
        for theta in [0.0, 1e-6, 0.2, 1.5, 3.0] {
            let omega = na::Vector6::new(0.7, -0.2, 0.4, theta, -theta * 0.2, theta * 0.1);
            let recovered = SE3::log_map(&SE3::exp_map(&omega));
            assert!(
                (recovered - omega).norm() < EPS,
                "SE3 chart-map error at theta={theta}: {}",
                (recovered - omega).norm()
            );
            assert_transform_close(
                SE3::exp(&SE3::log(&SE3::exp_map(&omega))),
                SE3::exp_map(&omega),
            );
        }
    }

    #[test]
    fn logarithm_handles_quaternion_double_cover() {
        let x = SE3::from_parts(
            na::Vector3::new(1.0, 2.0, 3.0),
            SO3::from_euler(&0.2, &-0.1, &0.3),
        );
        let q = x.rotation();
        let negated = SE3::from_parts(
            x.translation(),
            SO3::from_quat(-q.w(), -q.x(), -q.y(), -q.z()),
        );
        assert!((SE3::log_map(&x) - SE3::log_map(&negated)).norm() < EPS);
    }

    #[test]
    fn plus_minus_and_scaling_round_trip() {
        let x = SE3::from_parts(
            na::Vector3::new(1.0, 2.0, 3.0),
            SO3::from_euler(&0.1, &0.2, &0.3),
        );
        let delta = na::Vector6::new(0.2, -0.1, 0.3, 0.05, -0.07, 0.09);
        assert!(((x + delta) - x - delta).norm() < EPS);
        assert_transform_close((x * 0.25) / 0.25, x);
    }

    #[test]
    #[should_panic(expected = "cannot divide an SE3 transform by zero")]
    fn division_by_zero_panics_in_all_builds() {
        let _ = SE3::<f64>::identity() / 0.0;
    }

    #[test]
    fn supports_f32_and_lossy_casts() {
        let x = SE3::<f64>::identity().cast::<f32>();
        assert_eq!(x, SE3::<f32>::identity());
        let _ = x * 0.5;
    }

    #[test]
    fn display_identifies_the_group() {
        assert!(SE3::<f64>::identity().to_string().contains("SE(3)"));
    }

    #[test]
    fn assignment_and_left_scalar_operators() {
        let delta = na::Vector6::new(0.1, -0.2, 0.3, 0.01, -0.02, 0.03);
        let mut x = SE3::<f64>::identity();
        assert_eq!(x[0], 0.0);
        assert_eq!(x.array(), x.elements());
        x *= SE3::identity();
        x += delta;
        let expected = SE3::exp_map(&delta);
        assert_transform_close(x, expected);
        assert_transform_close(0.5 * x, x * 0.5);
        x *= 0.5;
        x /= 0.5;
        assert_transform_close(x, expected);
    }
}
