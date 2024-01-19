extern crate nalgebra as na;
use std::ops::Mul;

/// SO3 implementation

pub struct SO3<T: na::Scalar + na::ComplexField> {
    arr: na::Unit<na::Vector4<T>>, // w, x, y, z
}

impl<T: na::Scalar + na::ComplexField> Default for SO3<T> {
    fn default() -> Self {
        Self { arr: na::Unit::new_unchecked(na::Vector4::new(na::convert(1.0), na::convert(0.0), na::convert(0.0), na::convert(0.0))) }
    }
}

impl<T: na::Scalar + na::ComplexField> SO3<T> {
    pub fn new(q: na::Unit<na::Vector4<T>>) -> SO3<T> {
        Self { arr: q }
    }
    pub fn w(&self) -> &T {
        return &self.arr[(0, 0)]
    }
    pub fn x(&self) -> &T {
        return &self.arr[(1, 0)]
    }
    pub fn y(&self) -> &T {
        return &self.arr[(2, 0)]
    }
    pub fn z(&self) -> &T {
        return &self.arr[(3, 0)]
    }
    pub fn fromAxisAngle(axis: na::Vector3<T>, angle: T) -> SO3<T> {
        let th2: T = angle / T::new(2.0);
        let xyz: na::Vector3<T> = (th2.clone().sin() / (axis.norm() as T)) * axis;
        let q_arr: na::Unit<na::Vector4<T>> = na::Unit::new_normalize(na::Vector4::new(
            th2.clone().cos(),
            xyz[0], xyz[1], xyz[2]
        ));
        return SO3{ arr: q_arr }
    }
    pub fn fromEuler(roll: T, pitch: T, yaw: T) -> SO3<T> {
        let q_roll: SO3<T> = SO3::fromAxisAngle(na::Vector3::new(1.0, 0.0, 0.0), roll);
        let q_pitch: SO3<T> = SO3::fromAxisAngle(na::Vector3::new(0.0, 1.0, 0.0), pitch);
        let q_yaw: SO3<T> = SO3::fromAxisAngle(na::Vector3::new(0.0, 0.0, 1.0), yaw);
        let q_euler: SO3<T> = q_yaw * q_pitch * q_roll;
        return q_euler
    }
    pub fn roll(&self) -> T {
        (1.0 - 2.0 * (self.x() * self.x() + self.y() * self.y())).atan2(2.0 * (self.w() * self.x() + self.y() * self.z()))
    }
    pub fn pitch(&self) -> T {
        let val: T = 2.0 * (self.w() * self.y() - self.x() * self.z());
        // hold at 90 degrees if invalid
        if val.abs() > 1.0 {
            return (1.0).copysign(val) * f64::PI / 2.0;
        }
        else {
            return val.asin();
        }
    }
    pub fn yaw(&self) -> T {
        (1.0 - 2.0 * (self.y() * self.y() + self.z() * self.z())).atan2(2.0 * (self.w() * self.z() + self.x() * self.y()))
    }
    pub fn otimes(&self, q: SO3<T>) -> SO3<T> {
        return SO3{ arr: na::Unit::new_normalize(na::Vector4::new(
            self.w() * q.w() - self.x() * q.x() - self.y() * q.y() - self.z() * q.z(),
            self.w() * q.x() + self.x() * q.w() + self.y() * q.z() - self.z() * q.y(),
            self.w() * q.y() - self.x() * q.z() + self.y() * q.w() + self.z() * q.x(),
            self.w() * q.z() + self.x() * q.y() - self.y() * q.x() + self.z() * q.w()
        )) }
    }
}

impl<T: na::Scalar + na::ComplexField> Mul for SO3<T> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        self.otimes(rhs);
    }
}
