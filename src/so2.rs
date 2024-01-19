// use rand::distributions::{Distribution, Standard};

extern crate nalgebra as na;
// use na::{Scalar, Unit, Vector1, Vector2, Vector3, Vector4, Vector6, Matrix2};

/// SO2 implementation

pub mod so2 {

struct SO2<T: na::Scalar + na::ComplexField> {
    arr: na::Unit<na::Vector2<T>>, // w, x
}

impl<T: na::Scalar + na::ComplexField> SO2<T> {
    pub fn random() -> SO2<T> 
    where rand::distributions::Standard: rand::distributions::Distribution<T>, {
        return SO2{ arr: na::Unit::new_normalize(na::Vector2::new_random()) };
    }
    pub fn identity() -> SO2<T> {
        return SO2{ arr: na::Unit::new_unchecked(na::Vector2::new(na::convert(1.0), na::convert(0.0)))}
    }
    pub fn fromAngle(angle: T) -> SO2<T> {
        let c: T = angle.clone().cos();
        let s: T = angle.clone().sin();
        return SO2 { arr: na::Unit::new_unchecked(na::Vector2::new(na::convert(c), na::convert(s)))}
    }
    pub fn fromR(m: na::Matrix2<T>) -> SO2<T> {
        return SO2 { arr: na::Unit::new_normalize(na::Vector2::new(m[(0, 0)].clone(), m[(1, 0)].clone()))}
    }
    // fromTwoUnitVectors
    pub fn fromComplex(qw: T, qx: T) -> SO2<T> {
        return SO2{ arr: na::Unit::new_normalize(na::Vector2::new(qw, qx)) };
    }
    pub fn w(&self) -> &T {
        return &self.arr[(0, 0)]
    }
    pub fn x(&self) -> &T {
        return &self.arr[(1, 0)]
    }
    // array
    // copy
    // R
    // inverse
    // invert
    // angle
    // otimes
    // ...
}

// Unit Tests
#[cfg(test)]
mod test {
    use super::*;
    use na::{Vector1, Vector2, Vector3, Vector4, Vector6, Matrix};

    static EPSILON: f64 = 0.000001;
    // static PI: f64 = 3.14159265358979323846264338327950288_f64;

    #[test]
    fn test_action() {
        let numTests = 1000;
        let mut i = 0;
        while i < numTests {
            let so2: SO2<f64> = SO2::random();
            let v2: Vector2<f64> = Vector2::new_random();
            let mut v3 = v2.clone();
            // ++++
            assert!(false);
            i = i + 1;
        }
    }

    #[test]
    fn test_so2_getters_tmp() {
        // let q: SO2<f64> = SO2::fromComplex(1.0, 0.0);
        let q: SO2<f64> = SO2::identity();
        assert!((q.w() - 1.0).abs() < EPSILON);
        assert!(q.x().abs() < EPSILON);
    }
}

} // end so2
