//! Lie-group representations for 2D and 3D robotics geometry.
//!
//! The crate follows the conventions used by `manif-geom-cpp`: quaternion
//! coefficients are scalar-first, tangent perturbations are applied on the
//! right, and `x - y` computes `Log(y.inverse() * x)`.

pub extern crate nalgebra as na;

pub mod se2;
pub mod se3;
pub mod so2;
pub mod so3;

pub use se2::SE2;
pub use se3::SE3;
pub use so2::SO2;
pub use so3::SO3;

/// Double-precision two-dimensional rotation.
pub type SO2d = SO2<f64>;
/// Double-precision two-dimensional rigid transform.
pub type SE2d = SE2<f64>;
/// Double-precision three-dimensional rotation.
pub type SO3d = SO3<f64>;
/// Double-precision three-dimensional rigid transform.
pub type SE3d = SE3<f64>;
