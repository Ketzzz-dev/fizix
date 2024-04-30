use nalgebra::{Matrix3 as M3, Matrix4 as M4, Point3 as P3, Vector3 as V3, Quaternion as Q};

pub type Vector3 = V3<f64>;
pub type Point3 = P3<f64>;
pub type RotationMatrix = M3<f64>;
pub type TransformMatrix = M4<f64>;
pub type Quaternion = Q<f64>;