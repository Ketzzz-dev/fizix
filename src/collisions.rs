use crate::dynamics::RigidBody;
use crate::math::{Vector3};

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
    pub offset: f64,
}

pub struct Cube {
    pub half_size: Vector3
}

pub struct CollisionDetector;
pub struct CollisionResolver;

impl CollisionDetector {
    pub fn sphere_sphere(
        sphere_a: &Sphere,
        body_a: &RigidBody,
        sphere_b: &Sphere,
        body_b: &RigidBody
    ) -> Option<CollisionData> {
        let delta = body_a.position - body_b.position;
        let distance_sq = delta.magnitude_sq();
        let radius_sum = sphere_a.radius + sphere_b.radius;

        if distance_sq > radius_sum * radius_sum { return None }

        let distance = distance_sq.sqrt();
        let normal = delta * (1.0 / distance);

        let contact = Contact {
            point: body_a.position + normal * sphere_a.radius,
            normal,
            penetration: radius_sum - distance
        };

        Some(CollisionData { contacts: vec![contact] })
    }

    pub fn sphere_plane(
        sphere_a: &Sphere,
        body_a: &RigidBody,
        plane_b: &Plane,
        _body_b: &RigidBody
    ) -> Option<CollisionData> {
        let distance_to_plane = body_a.position.dot(&plane_b.normal) - plane_b.offset;

        if distance_to_plane > sphere_a.radius { return None }

        let contact = Contact {
            point: body_a.position - plane_b.normal * (distance_to_plane - sphere_a.radius),
            normal: plane_b.normal,
            penetration: sphere_a.radius - distance_to_plane
        };

        Some(CollisionData { contacts: vec![contact] })
    }

    pub fn cube_plane(
        cube_a: &Cube,
        body_a: &RigidBody,
        plane_b: &Plane,
        _body_b: &RigidBody
    ) -> Option<CollisionData> {
        let mut contacts = vec![];
        let mut penetration = f64::MAX;

        for x in &[-1.0, 1.0] {
            for y in &[-1.0, 1.0] {
                for z in &[-1.0, 1.0] {
                    let offset = Vector3 {
                        x: x * cube_a.half_size.x,
                        y: y * cube_a.half_size.y,
                        z: z * cube_a.half_size.z
                    };

                    let world_point = body_a.transform * offset;
                    let distance_to_plane = world_point.dot(&plane_b.normal) - plane_b.offset;

                    if distance_to_plane > 0.0 { return None }
                    if distance_to_plane > penetration { continue }

                    let contact = Contact {
                        point: world_point - plane_b.normal * distance_to_plane,
                        normal: plane_b.normal,
                        penetration: -distance_to_plane
                    };

                    penetration = distance_to_plane;
                    contacts.push(contact);
                }
            }
        }

        Some(CollisionData { contacts })
    }

    pub fn cube_sphere(
        cube_a: &Cube,
        body_a: &RigidBody,
        sphere_b: &Sphere,
        body_b: &RigidBody
    ) -> Option<CollisionData> {
        let mut contacts = vec![];
        let mut penetration = f64::MAX;

        for x in &[-1.0, 1.0] {
            for y in &[-1.0, 1.0] {
                for z in &[-1.0, 1.0] {
                    let offset = Vector3 {
                        x: x * cube_a.half_size.x,
                        y: y * cube_a.half_size.y,
                        z: z * cube_a.half_size.z
                    };

                    let world_point = body_a.transform * offset;
                    let delta = world_point - body_b.position;
                    let distance_sq = delta.magnitude_sq();
                    let radius_sum = sphere_b.radius;

                    if distance_sq > radius_sum * radius_sum { return None }

                    let distance = distance_sq.sqrt();
                    let normal = delta * (1.0 / distance);

                    let contact = Contact {
                        point: body_b.position + normal * sphere_b.radius,
                        normal,
                        penetration: radius_sum - distance
                    };

                    if contact.penetration < penetration {
                        penetration = contact.penetration;
                        contacts.clear();
                    }

                    contacts.push(contact);
                }
            }
        }

        Some(CollisionData { contacts })
    }
}

impl CollisionResolver {
    pub fn resolve_collision(
        body_a: &mut RigidBody,
        body_b: &mut RigidBody,
        data: &CollisionData
    ) {
        let total_inverse_mass = body_a.inverse_mass + body_b.inverse_mass;
        let relative_velocity = body_a.linear_velocity - body_b.linear_velocity;

        for contact in &data.contacts {
            let normal_velocity = relative_velocity.dot(&contact.normal);

            if normal_velocity > 0.0 { return }

            let restitution = 0.5;
            let impulse = -(1.0 + restitution) * normal_velocity / total_inverse_mass;
            let impulse_vector = impulse * contact.normal;

            body_a.linear_velocity += impulse_vector * body_a.inverse_mass;
            body_b.linear_velocity -= impulse_vector * body_b.inverse_mass;
        }
    }

    pub fn resolve_penetration(
        body_a: &mut RigidBody,
        body_b: &mut RigidBody,
        data: &CollisionData
    ) {
        let total_inverse_mass = body_a.inverse_mass + body_b.inverse_mass;

        for contact in &data.contacts {
            let move_per_mass = contact.normal * (contact.penetration / total_inverse_mass);

            body_a.position += move_per_mass * body_a.inverse_mass;
            body_b.position -= move_per_mass * body_b.inverse_mass;
        }
    }
}