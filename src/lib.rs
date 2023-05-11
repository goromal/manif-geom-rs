//! Rust implementation of the manifold representations in manif-geom-rs
 
extern crate nalgebra as na;

use na::{Vector1, Vector2, Vector3, Vector4, Vector6, Matrix};

// Implemented manifold types
struct SO2<T> {
    w: T,
    x: T,
}
struct SE2<T> {
    t: Vector2<T>,
    q: SO2<T>,
}
struct SO3<T> {
    w: T,
    v: Vector3<T>,
}
struct SE3<T> {
    t: Vector3<T>,
    q: SO3<T>,
}

// Shared manifold traits
// TODO

// SO2-specific methods
impl<T> SO2<T> {
    fn w(&self) -> &T {
        &self.w
    }
    fn x(&self) -> &T {
        &self.x
    }
}

// Tests
#[cfg(test)]
mod test {
    use super::*;
    use na::{Vector1, Vector2, Vector3, Vector4, Vector6, Matrix};

    static EPSILON: f64 = 0.000001;
    static PI: f64 = 3.14159265358979323846264338327950288_f64;

    #[test]
    fn test_so2_getters_tmp() {
        let q: SO2<f64> = { SO2 { w: 1.0, x: 0.0 } };
        assert!((q.w() - 1.0).abs() < EPSILON);
        assert!(q.x().abs() < EPSILON);
    }
}
