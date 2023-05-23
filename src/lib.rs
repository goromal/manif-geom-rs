//! Rust implementation of the manifold representations in manif-geom-rs

// use num_traits::Float; TODO remove?

use rand::distributions::{Distribution, Standard};

extern crate nalgebra as na;
use na::{Scalar, Unit, Vector1, Vector2, Vector3, Vector4, Vector6, Matrix};

/// Implemented manifold types : Float + na::ComplexField

struct SO2<T: Scalar + na::ComplexField> {
    arr: Unit<Vector2<T>>, // w, x
}
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

/// SO2 implementation

impl<T: Scalar + na::ComplexField> SO2<T> {
    pub fn random() -> SO2<T> 
    where Standard: Distribution<T>, {
        return SO2{ arr: Unit::new_normalize(Vector2::new_random()) };
    }
    pub fn fromComplex(qw: T, qx: T) -> SO2<T> {
        return SO2{ arr: Unit::new_normalize(Vector2::new(qw, qx)) };
    }
    pub fn w(&self) -> &T {
        return &self.arr[(0, 0)]
    }
    pub fn x(&self) -> &T {
        return &self.arr[(1, 0)]
    }
}

// Tests
#[cfg(test)]
mod test {
    use super::*;
    use na::{Vector1, Vector2, Vector3, Vector4, Vector6, Matrix};

    static EPSILON: f64 = 0.000001;
    // static PI: f64 = 3.14159265358979323846264338327950288_f64;

    #[test]
    fn test_so2_getters_tmp() {
        let q: SO2<f64> = SO2::fromComplex(1.0, 0.0);
        assert!((q.w() - 1.0).abs() < EPSILON);
        assert!(q.x().abs() < EPSILON);
    }
}
