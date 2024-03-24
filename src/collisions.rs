use crate::dynamics::RigidBody;
use crate::math::{Matrix4x4, Vector3};

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

pub struct Box {
    pub half_size: Vector3
}

pub struct CollisionDetector;

impl CollisionDetector {
    pub fn sphere_sphere(
        primitive_a: &Sphere,
        body_a: &RigidBody,
        primitive_b: &Sphere,
        body_b: &RigidBody
    ) -> Option<CollisionData> {
        let midline = body_a.position - body_b.position;
        let distance_sq = midline.magnitude_sq();
        let radii = primitive_a.radius + primitive_b.radius;

        if distance_sq <= 0.0 || distance_sq >= radii * radii {
            return None;
        }

        let distance = distance_sq.sqrt();

        let contact = Contact {
            point: body_a.position + midline * 0.5,
            normal: midline * (1.0 / distance),

            penetration: radii - distance
        };

        Some(CollisionData { contacts: vec![contact] })
    }

    pub fn sphere_plane(
        primitive_a: &Sphere,
        body_a: &RigidBody,
        primitive_b: &Plane,
        body_b: &RigidBody
    ) -> Option<CollisionData> {
        let center_distance = primitive_b.normal.dot(&body_a.position) - primitive_b.offset;

        if center_distance * center_distance > primitive_a.radius * primitive_a.radius {
            return None;
        }

        let (normal, penetration) = if center_distance < 0.0 {
            (-primitive_b.normal, primitive_a.radius + center_distance)
        } else {
            (primitive_b.normal, primitive_a.radius - center_distance)
        };

        let contact = Contact {
            point: body_a.position - primitive_b.normal * center_distance,
            normal,
            penetration
        };

        Some(CollisionData { contacts: vec![contact] })
    }

    pub fn box_plane(
        primitive_a: &Box,
        body_a: &RigidBody,
        primitive_b: &Plane,
        body_b: &RigidBody
    ) -> Option<CollisionData> {
        let vertices = [
            Vector3 { x: -primitive_a.half_size.x, y: -primitive_a.half_size.y, z: -primitive_a.half_size.z },
            Vector3 { x: -primitive_a.half_size.x, y: -primitive_a.half_size.y, z: primitive_a.half_size.z },
            Vector3 { x: -primitive_a.half_size.x, y: primitive_a.half_size.y, z: -primitive_a.half_size.z },
            Vector3 { x: -primitive_a.half_size.x, y: primitive_a.half_size.y, z: primitive_a.half_size.z },
            Vector3 { x: primitive_a.half_size.x, y: -primitive_a.half_size.y, z: -primitive_a.half_size.z },
            Vector3 { x: primitive_a.half_size.x, y: -primitive_a.half_size.y, z: primitive_a.half_size.z },
            Vector3 { x: primitive_a.half_size.x, y: primitive_a.half_size.y, z: -primitive_a.half_size.z },
            Vector3 { x: primitive_a.half_size.x, y: primitive_a.half_size.y, z: primitive_a.half_size.z }
        ];

        for vertex in vertices {
            let vertex = body_a.transform * vertex;
            let distance = vertex.dot(&primitive_b.normal);

            if distance <= primitive_b.offset {
                let contact = Contact {
                    point: primitive_b.normal * (distance - primitive_b.offset) + vertex,
                    normal: primitive_b.normal,
                    penetration: primitive_b.offset - distance
                };

                return Some(CollisionData {
                    contacts: vec![contact]
                })
            }
        }

        None
    }

    pub fn box_sphere(
        primitive_a: &Box,
        body_a: &RigidBody,
        primitive_b: &Sphere,
        body_b: &RigidBody
    ) -> Option<CollisionData> {
        let relative_position = body_a.transform.inverse() * body_b.position;

        if
            relative_position.x.abs() - primitive_b.radius > primitive_a.half_size.x ||
            relative_position.y.abs() - primitive_b.radius > primitive_a.half_size.y ||
            relative_position.z.abs() - primitive_b.radius > primitive_a.half_size.z
        { return None }

        let mut closest_point = Vector3::ZERO;
        let mut distance = relative_position.x;

        if distance > primitive_a.half_size.x { distance = primitive_a.half_size.x }
        if distance < -primitive_a.half_size.x { distance = -primitive_a.half_size.x }

        closest_point.x = distance;
        distance = relative_position.y;

        if distance > primitive_a.half_size.y { distance = primitive_a.half_size.y }
        if distance < -primitive_a.half_size.y { distance = -primitive_a.half_size.y }

        closest_point.y = distance;
        distance = relative_position.z;

        if distance > primitive_a.half_size.z { distance = primitive_a.half_size.z }
        if distance < -primitive_a.half_size.z { distance = -primitive_a.half_size.z }

        closest_point.z = distance;

        let distance = (closest_point - relative_position).magnitude_sq();

        if distance > primitive_b.radius * primitive_b.radius { return None }

        let closest_point = body_a.transform * closest_point;

        let contact = Contact {
            point: closest_point,
            normal: (body_b.position - closest_point).normalized(),
            penetration: primitive_b.radius - distance.sqrt()
        };

        Some(CollisionData { contacts: vec![contact] })
    }

    pub fn box_box(
        primitive_a: &Box,
        body_a: &RigidBody,
        primitive_b: &Box,
        body_b: &RigidBody
    ) -> Option<CollisionData> {
        None
    }

    fn transform_to_axis(primitve: &Box, transform: &Matrix4x4, axis: &Vector3) -> f64 {
        let c1 = Vector3 { x: transform.m11, y: transform.m21, z: transform.m31 };
        let c2 = Vector3 { x: transform.m12, y: transform.m22, z: transform.m32 };
        let c3 = Vector3 { x: transform.m13, y: transform.m23, z: transform.m33 };

        primitve.half_size.x * axis.dot(&c1).abs() +
            primitve.half_size.y * axis.dot(&c2).abs() +
            primitve.half_size.z * axis.dot(&c3).abs()
    }

    fn overlap_on_axis(
        primitive_a: &Box,
        body_a: &RigidBody,
        primitive_b: &Box,
        body_b: &RigidBody,
        axis: &Vector3
    ) -> bool {
        let projection_a = Self::transform_to_axis(primitive_a, &body_a.transform, axis);
        let projection_b = Self::transform_to_axis(primitive_b, &body_b.transform, axis);

        let distance = (body_b.position - body_a.position).dot(axis).abs();

        distance < projection_a + projection_b
    }
}