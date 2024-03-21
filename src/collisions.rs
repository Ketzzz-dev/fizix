use crate::math::Vector3;

pub struct Contact {
    pub point: Vector3,
    pub normal: Vector3,

    pub penetration: f64
}