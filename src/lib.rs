//! Rust implementation of the manifold representations in manif-geom-rs

// use num_traits::Float; TODO remove?

mod so2;

use rand::distributions::{Distribution, Standard};

extern crate nalgebra as na;
use na::{Scalar, Unit, Vector1, Vector2, Vector3, Vector4, Vector6, Matrix2};

/// Implemented manifold types : Float + na::ComplexField
/*
struct SE2<T: Scalar + na::ComplexField> {
    t: Vector2<T>,
    q: SO2<T>,
}
struct SO3<T: Scalar + na::ComplexField> {
    arr: Unit<Vector4<T>>, // w, x, y, z
}
struct SE3<T: Scalar + na::ComplexField> {
    t: Vector3<T>,
    q: SO3<T>,
}
*/
