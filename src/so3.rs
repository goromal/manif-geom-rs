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

impl<T: na::Scalar + na::ComplexField + na::RealField> SO3<T> {
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
    pub fn from_axis_angle(axis: &na::Vector3<T>, angle: &T) -> SO3<T> {
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
    pub fn from_euler(roll: &T, pitch: &T, yaw: &T) -> SO3<T> {
        let q_roll: SO3<T> = SO3::from_axis_angle(&na::Vector3::x(), roll);
        let q_pitch: SO3<T> = SO3::from_axis_angle(&na::Vector3::y(), pitch);
        let q_yaw: SO3<T> = SO3::from_axis_angle(&na::Vector3::z(), yaw);
        let q_euler: SO3<T> = q_yaw * q_pitch * q_roll;
        q_euler
    }
    pub fn roll(&self) -> T {
        let x: T = self.x().clone();
        let y: T = self.y().clone();
        let z: T = self.z().clone();
        let w: T = self.w().clone();
        ((w.clone() * x.clone() + y.clone() * z.clone()) * na::convert(2.0)).atan2(T::one() - (x.clone() * x.clone() + y.clone() * y.clone()) * na::convert(2.0))
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
        ((w.clone() * z.clone() + x.clone() * y.clone()) * na::convert(2.0)).atan2(T::one() - (y.clone() * y.clone() + z.clone() * z.clone()) * na::convert(2.0))
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

impl<T: na::Scalar + na::ComplexField + na::RealField> Mul<SO3<T>> for SO3<T> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        self.otimes(rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField> Mul<na::Vector3<T>> for SO3<T> {
    type Output = na::Vector3<T>;
    fn mul(self, rhs: na::Vector3<T>) -> na::Vector3<T> {
        let vx: T = rhs[0].clone();
        let vy: T = rhs[1].clone();
        let vz: T = rhs[2].clone();
        let qxx: T = self.x().clone() * self.x().clone();
        let qxy: T = self.x().clone() * self.y().clone();
        let qxz: T = self.x().clone() * self.z().clone();
        let qyy: T = self.y().clone() * self.y().clone();
        let qyz: T = self.y().clone() * self.z().clone();
        let qzz: T = self.z().clone() * self.z().clone();
        let qwx: T = self.w().clone() * self.x().clone();
        let qwy: T = self.w().clone() * self.y().clone();
        let qwz: T = self.w().clone() * self.z().clone();

        na::Vector3::new(
            (T::one() - qyy.clone() * na::convert(2.0) - qzz.clone() * na::convert(2.0)) * vx.clone() +
            (qxy.clone() * na::convert(2.0) - qwz.clone() * na::convert(2.0)) * vy.clone() +
            (qxz.clone() + qwy.clone()) * na::convert(2.0) * vz.clone(),
            (qxy.clone() + qwz.clone()) * na::convert(2.0) * vx.clone() +
            (T::one() - (qxx.clone() + qzz.clone()) * na::convert(2.0)) * vy.clone() +
            (qyz.clone() - qwx.clone()) * na::convert(2.0) * vz.clone(),
            (qxz.clone() - qwy.clone()) * na::convert(2.0) * vx.clone() +
            (qyz.clone() + qwx.clone()) * na::convert(2.0) * vy.clone() +
            (T::one() - (qxx.clone() + qyy.clone()) * na::convert(2.0)) * vz.clone(),
        )
    }
}
