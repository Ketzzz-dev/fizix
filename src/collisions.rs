use log::warn;
use crate::dynamics::RigidBody;
use crate::math::{Matrix3x3, Matrix4x4, Vector3};

pub struct Contact {
    pub point: Vector3,
    pub normal: Vector3,

    pub penetration: f64
}

pub struct CollisionData {
    pub contacts: Vec<Contact>
}

pub struct Sphere {
    pub radius: f64
}
pub struct Plane {
    pub normal: Vector3,
    pub offset: f64
}

pub trait Collider {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3x3;
    fn collides(&self, this_transform: &Matrix4x4, other: &impl Collider, other_transform: &Matrix4x4) -> Option<CollisionData>;

    fn collides_with_sphere(&self, this_transform: &Matrix4x4, sphere: &Sphere, other_transform: &Matrix4x4) -> Option<CollisionData>;
    fn collides_with_plane(&self, this_transform: &Matrix4x4, plane: &Plane) -> Option<CollisionData>;
}

impl Collider for Sphere {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3x3 {
        let inverse_inertia = 5.0 / (2.0 * mass * self.radius * self.radius);

        Matrix3x3 {
            m11: inverse_inertia, m12: 0.0, m13: 0.0,
            m21: 0.0, m22: inverse_inertia, m23: 0.0,
            m31: 0.0, m32: 0.0, m33: inverse_inertia
        }
    }
    fn collides(&self, this_transform: &Matrix4x4, other: &impl Collider, other_transform: &Matrix4x4) -> Option<CollisionData> {
        other.collides_with_sphere(other_transform, self, this_transform)
    }

    fn collides_with_sphere(&self, this_transform: &Matrix4x4, sphere: &Sphere, other_transform: &Matrix4x4) -> Option<CollisionData> {
        sphere_vs_sphere(self, this_transform, sphere, other_transform)
    }

    fn collides_with_plane(&self, this_transform: &Matrix4x4, plane: &Plane) -> Option<CollisionData> {
        sphere_vs_plane(self, this_transform, plane)
    }
}

impl Collider for Plane {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3x3 {
        Matrix3x3::ZERO
    }

    fn collides(&self, this_transform: &Matrix4x4, other: &impl Collider, other_transform: &Matrix4x4) -> Option<CollisionData> {
        other.collides_with_plane(other_transform, self)
    }

    fn collides_with_sphere(&self, this_transform: &Matrix4x4, sphere: &Sphere, other_transform: &Matrix4x4) -> Option<CollisionData> {
        return if let Some(mut collision_data) = sphere_vs_plane(sphere, other_transform, self) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_plane(&self, this_transform: &Matrix4x4, plane: &Plane) -> Option<CollisionData> {
        warn!("Plane vs Plane collision is not supported");

        None
    }
}

impl CollisionData {
    pub fn solve(&self, body_a: &mut RigidBody, body_b: &mut RigidBody) {
        let total_inverse_mass = body_a.inverse_mass + body_b.inverse_mass;
        let relative_velocity = body_a.linear_velocity - body_b.linear_velocity;

        for contact in &self.contacts {
            let move_per_mass = contact.normal * (contact.penetration / total_inverse_mass);

            if body_a.inverse_mass > 0.0 {
                body_a.position -= move_per_mass * body_a.inverse_mass;
            }
            if body_b.inverse_mass > 0.0 {
                body_b.position += move_per_mass * body_b.inverse_mass;
            }

            let normal_velocity = relative_velocity.dot(&contact.normal);

            if normal_velocity <= 0.0 { return; }

            let impulse = -1.5 * normal_velocity / total_inverse_mass;

            if body_a.inverse_mass > 0.0 {
                body_a.linear_velocity += impulse * contact.normal * body_a.inverse_mass;
            }
            if body_b.inverse_mass > 0.0 {
                body_b.linear_velocity -= impulse * contact.normal * body_b.inverse_mass;
            }
        }
    }
}

fn sphere_vs_sphere(
    sphere_a: &Sphere,
    transform_a: &Matrix4x4,
    sphere_b: &Sphere,
    transform_b: &Matrix4x4
) -> Option<CollisionData> {
    let center_a = transform_a.translation();
    let center_b = transform_b.translation();

    let delta = center_a - center_b;
    let distance_sq = delta.magnitude_sq();
    let radius_sum = sphere_a.radius + sphere_b.radius;

    if distance_sq > radius_sum * radius_sum { return None; }

    let distance = distance_sq.sqrt();
    let normal = delta * (1.0 / distance);

    let contact = Contact {
        point: center_a + normal * sphere_a.radius,
        normal,
        penetration: radius_sum - distance
    };

    Some(CollisionData { contacts: vec![contact] })
}

fn sphere_vs_plane(
    sphere: &Sphere,
    transform_sphere: &Matrix4x4,
    plane: &Plane
) -> Option<CollisionData> {
    let sphere_center = transform_sphere.translation();
    let distance_to_plane = sphere_center.dot(&plane.normal) - plane.offset;

    if distance_to_plane > sphere.radius { return None; }

    let contact = Contact {
        point: sphere_center - plane.normal * (distance_to_plane - sphere.radius),
        normal: plane.normal,
        penetration: sphere.radius - distance_to_plane
    };

    Some(CollisionData { contacts: vec![contact] })
}