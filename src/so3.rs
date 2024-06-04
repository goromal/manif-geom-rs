extern crate nalgebra as na;
use std::ops::Mul;

/// SO3 implementation

pub struct SO3<T: na::Scalar + na::ComplexField + na::RealField + Copy> {
    arr: na::Unit<na::Vector4<T>>, // w, x, y, z
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Default for SO3<T> {
    fn default() -> Self {
        Self { arr: na::Unit::new_unchecked(na::Vector4::new(na::convert(1.0), na::convert(0.0), na::convert(0.0), na::convert(0.0))) }
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> SO3<T> {
    pub fn new(q: na::Unit<na::Vector4<T>>) -> SO3<T> {
        Self { arr: q }
    }
    pub fn w(&self) -> T {
        self.arr[(0, 0)]
    }
    pub fn x(&self) -> T {
        self.arr[(1, 0)]
    }
    pub fn y(&self) -> T {
        self.arr[(2, 0)]
    }
    pub fn z(&self) -> T {
        self.arr[(3, 0)]
    }
    pub fn from_axis_angle(axis: &na::Vector3<T>, angle: &T) -> SO3<T> {
        let angle_scale: T = T::one() / na::convert(2.0);
        let th2: T = angle_scale * *angle;
        let axis_normalized: na::Vector3<T> = axis.normalize();
        let scale: T = th2.sin();
        let q_arr: na::Unit<na::Vector4<T>> = na::Unit::new_normalize(na::Vector4::new(
            th2.cos(),
            scale * axis_normalized[0],
            scale * axis_normalized[1],
            scale * axis_normalized[2]
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
        let x: T = self.x();
        let y: T = self.y();
        let z: T = self.z();
        let w: T = self.w();
        ((w * x + y * z) * na::convert(2.0)).atan2(T::one() - (x * x + y * y) * na::convert(2.0))
    }
    pub fn pitch(&self) -> T {
        let x: T = self.x();
        let y: T = self.y();
        let z: T = self.z();
        let w: T = self.w();
        let val: T =  (w * y - x * z) * na::convert(2.0);
        // hold at 90 degrees if invalid
        if val.abs() > na::convert(1.0) {
            return T::one().copysign(val) * na::convert(std::f64::consts::PI / 2.0);
        }
        else {
            return val.asin();
        }
    }
    pub fn yaw(&self) -> T {
        let x: T = self.x();
        let y: T = self.y();
        let z: T = self.z();
        let w: T = self.w();
        ((w * z + x * y) * na::convert(2.0)).atan2(T::one() - (y * y + z * z) * na::convert(2.0))
    }
    pub fn otimes(&self, q: SO3<T>) -> SO3<T> {
        let self_x: T = self.x();
        let self_y: T = self.y();
        let self_z: T = self.z();
        let self_w: T = self.w();
        let q_x: T = q.x();
        let q_y: T = q.y();
        let q_z: T = q.z();
        let q_w: T = q.w();
        SO3{ arr: na::Unit::new_normalize(na::Vector4::new(
            self_w * q_w - self_x * q_x - self_y * q_y - self_z * q_z,
            self_w * q_x + self_x * q_w + self_y * q_z - self_z * q_y,
            self_w * q_y - self_x * q_z + self_y * q_w + self_z * q_x,
            self_w * q_z + self_x * q_y - self_y * q_x + self_z * q_w
        )) }
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<SO3<T>> for SO3<T> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        self.otimes(rhs)
    }
}

impl<T: na::Scalar + na::ComplexField + na::RealField + Copy> Mul<na::Vector3<T>> for SO3<T> {
    type Output = na::Vector3<T>;
    fn mul(self, rhs: na::Vector3<T>) -> na::Vector3<T> {
        let vx: T = rhs[0];
        let vy: T = rhs[1];
        let vz: T = rhs[2];
        let qxx: T = self.x() * self.x();
        let qxy: T = self.x() * self.y();
        let qxz: T = self.x() * self.z();
        let qyy: T = self.y() * self.y();
        let qyz: T = self.y() * self.z();
        let qzz: T = self.z() * self.z();
        let qwx: T = self.w() * self.x();
        let qwy: T = self.w() * self.y();
        let qwz: T = self.w() * self.z();

        na::Vector3::new(
            (T::one() - qyy * na::convert(2.0) - qzz * na::convert(2.0)) * vx +
            (qxy * na::convert(2.0) - qwz * na::convert(2.0)) * vy +
            (qxz + qwy) * na::convert(2.0) * vz,
            (qxy + qwz) * na::convert(2.0) * vx +
            (T::one() - (qxx + qzz) * na::convert(2.0)) * vy +
            (qyz - qwx) * na::convert(2.0) * vz,
            (qxz - qwy) * na::convert(2.0) * vx +
            (qyz + qwx) * na::convert(2.0) * vy +
            (T::one() - (qxx + qyy) * na::convert(2.0)) * vz,
        )
    }
}
