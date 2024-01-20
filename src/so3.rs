extern crate nalgebra as na;
use std::ops::Mul;

/// SO3 implementation

pub struct SO3<T: na::Scalar + na::ComplexField + na::RealField> {
    arr: na::Unit<na::Vector4<T>>, // w, x, y, z
}

impl<T: na::Scalar + na::ComplexField + na::RealField> Default for SO3<T> {
    fn default() -> Self {
        Self { arr: na::Unit::new_unchecked(na::Vector4::new(na::convert(1.0), na::convert(0.0), na::convert(0.0), na::convert(0.0))) }
    }
}

impl<'a, 'b: 'a, T: na::Scalar + na::ComplexField + na::RealField> SO3<T> where &'b T: Mul<&'a T, Output = T> {
    pub fn new(q: na::Unit<na::Vector4<T>>) -> SO3<T> {
        Self { arr: q }
    }
    pub fn w(&self) -> &T {
        &self.arr[(0, 0)]
    }
    pub fn x(&self) -> &T {
        &self.arr[(1, 0)]
    }
    pub fn y(&self) -> &T {
        &self.arr[(2, 0)]
    }
    pub fn z(&self) -> &T {
        &self.arr[(3, 0)]
    }
    pub fn fromAxisAngle(axis: &na::Vector3<T>, angle: &T) -> SO3<T> {
        let angle_scale: T = T::one() / na::convert(2.0);
        let th2: T = angle_scale * angle.clone();
        let axis_normalized: na::Vector3<T> = axis.normalize();
        let scale: T = th2.clone().sin();
        let q_arr: na::Unit<na::Vector4<T>> = na::Unit::new_normalize(na::Vector4::new(
            th2.clone().cos(),
            scale.clone() * axis_normalized[0].clone(),
            scale.clone() * axis_normalized[1].clone(),
            scale.clone() * axis_normalized[2].clone()
        ));
        SO3{ arr: q_arr }
    }
    pub fn fromEuler(roll: &T, pitch: &T, yaw: &T) -> SO3<T> {
        let q_roll: SO3<T> = SO3::fromAxisAngle(&na::Vector3::x(), roll);
        let q_pitch: SO3<T> = SO3::fromAxisAngle(&na::Vector3::y(), pitch);
        let q_yaw: SO3<T> = SO3::fromAxisAngle(&na::Vector3::z(), yaw);
        let q_euler: SO3<T> = q_yaw * q_pitch * q_roll;
        q_euler
    }
    pub fn roll(&self) -> T {
        let x: T = self.x().clone();
        let y: T = self.y().clone();
        let z: T = self.z().clone();
        let w: T = self.w().clone();
        (T::one() - (x.clone() * x.clone() + y.clone() * y.clone()) * na::convert(2.0)).atan2((w.clone() * x.clone() + y.clone() * z.clone()) * na::convert(2.0))
    }
    pub fn pitch(&self) -> T {
        let x: T = self.x().clone();
        let y: T = self.y().clone();
        let z: T = self.z().clone();
        let w: T = self.w().clone();
        let val: T =  (w.clone() * y.clone() - x.clone() * z.clone()) * na::convert(2.0);
        // hold at 90 degrees if invalid
        if val.clone().abs() > na::convert(1.0) {
            return T::one().copysign(val) * na::convert(std::f64::consts::PI / 2.0);
        }
        else {
            return val.asin();
        }
    }
    pub fn yaw(&self) -> T {
        let x: T = self.x().clone();
        let y: T = self.y().clone();
        let z: T = self.z().clone();
        let w: T = self.w().clone();
        (T::one() - (y.clone() * y.clone() + z.clone() * z.clone()) * na::convert(2.0)).atan2((w.clone() * z.clone() + x.clone() * y.clone()) * na::convert(2.0))
    }
    pub fn otimes(&self, q: SO3<T>) -> SO3<T> {
        let self_x: T = self.x().clone();
        let self_y: T = self.y().clone();
        let self_z: T = self.z().clone();
        let self_w: T = self.w().clone();
        let q_x: T = q.x().clone();
        let q_y: T = q.y().clone();
        let q_z: T = q.z().clone();
        let q_w: T = q.w().clone();
        SO3{ arr: na::Unit::new_normalize(na::Vector4::new(
            self_w.clone() * q_w.clone() - self_x.clone() * q_x.clone() - self_y.clone() * q_y.clone() - self_z.clone() * q_z.clone(),
            self_w.clone() * q_x.clone() + self_x.clone() * q_w.clone() + self_y.clone() * q_z.clone() - self_z.clone() * q_y.clone(),
            self_w.clone() * q_y.clone() - self_x.clone() * q_z.clone() + self_y.clone() * q_w.clone() + self_z.clone() * q_x.clone(),
            self_w.clone() * q_z.clone() + self_x.clone() * q_y.clone() - self_y.clone() * q_x.clone() + self_z.clone() * q_w.clone()
        )) }
    }
}

impl<'a, 'b: 'a, T: na::Scalar + na::ComplexField + na::RealField> Mul for SO3<T> where &'b T: Mul<&'a T, Output = T> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        self.otimes(rhs)
    }
}
