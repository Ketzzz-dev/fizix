use nalgebra::{Matrix3};
use crate::Precision;

pub fn cuboid_inertia_tensor(width: Precision, height: Precision, length: Precision, mass: Precision) -> Matrix3<Precision> {
    let x2 = width * width;
    let y2 = height * height;
    let z2 = length * length;

    let i = mass / 12.0;

    Matrix3::new(
        i * (y2 + z2), 0.0, 0.0,
        0.0, i * (x2 + z2), 0.0,
        0.0, 0.0, i * (x2 + y2)
    )
}

pub fn sphere_inertia_tensor(radius: Precision, mass: Precision) -> Matrix3<Precision> {
    let i = 2.0 * mass * radius * radius / 5.0;

    Matrix3::new(
        i, 0.0, 0.0,
        0.0, i, 0.0,
        0.0, 0.0, i
    )
}

pub fn cylinder_inertia_tensor(radius: Precision, height: Precision, mass: Precision) -> Matrix3<Precision> {
    let i_xz = mass * (3.0 * radius * radius + height * height) / 12.0;

    Matrix3::new(
        i_xz, 0.0, 0.0,
        0.0, 0.5 * mass * radius * radius, 0.0,
        0.0, 0.0, i_xz
    )
}