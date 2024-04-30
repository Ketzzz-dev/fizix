pub mod narrow_phase;

use crate::aliases::{Point3, RotationMatrix, Vector3};

pub enum Collider {
    Sphere { radius: f64 },
    Plane {
        normal: Vector3,
        offset: f64
    },
    Cuboid {
        half_extents: Vector3,
        local_vertices: [Point3; 8]
    }
}

impl Collider {
    pub fn inverse_inertia_tensor(&self, mass: f64) -> RotationMatrix {
        match self {
            Collider::Sphere { radius } => {
                let inertia = 2.0 * mass * radius * radius / 5.0;
                let inertia_tensor = RotationMatrix::from_diagonal_element(inertia);

                inertia_tensor.try_inverse().unwrap_or(RotationMatrix::zeros())
            }
            Collider::Plane { .. } => {
                RotationMatrix::zeros()
            }
            Collider::Cuboid { half_extents, .. } => {
                let extents = 2.0 * half_extents;

                let xx = extents.x * extents.x;
                let yy = extents.y * extents.y;
                let zz = extents.z * extents.z;

                let inertia_diagonal = mass * Vector3::new(yy + zz, xx + zz, xx + yy) / 12.0;
                let inertia_tensor = RotationMatrix::from_diagonal(&inertia_diagonal);

                inertia_tensor.try_inverse().unwrap_or(RotationMatrix::zeros())
            }
        }
    }
}
impl Collider {
    pub fn sphere(radius: f64) -> Self {
        Collider::Sphere { radius }
    }

    pub fn plane(normal: Vector3, offset: f64) -> Self {
        Collider::Plane { normal, offset }
    }

    pub fn cuboid(half_extents: Vector3) -> Self {
        let x = half_extents.x;
        let y = half_extents.y;
        let z = half_extents.z;

        let vertices = [
            Point3::new(-x, -y, -z),
            Point3::new(-x, -y, z),
            Point3::new(-x, y, -z),
            Point3::new(-x, y, z),
            Point3::new(x, -y, -z),
            Point3::new(x, -y, z),
            Point3::new(x, y, -z),
            Point3::new(x, y, z)
        ];

        Collider::Cuboid { half_extents, local_vertices: vertices }
    }
}