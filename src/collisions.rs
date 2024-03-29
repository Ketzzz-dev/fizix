use log::warn;
use crate::dynamics::RigidBody;
use crate::math::{Matrix3x3, Matrix4x4, Vector3};

pub const RESTITUTION: f64 = 0.75;

pub trait Collider {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3x3;
    fn collides(&self, this_transform: &Matrix4x4, other: &impl Collider, other_transform: &Matrix4x4) -> Option<CollisionData>;

    fn collides_with_sphere(&self, this_transform: &Matrix4x4, sphere: &Sphere, sphere_transform: &Matrix4x4) -> Option<CollisionData>;
    fn collides_with_cuboid(&self, this_transform: &Matrix4x4, cuboid: &Cuboid, cuboid_transform: &Matrix4x4) -> Option<CollisionData>;
    fn collides_with_plane(&self, this_transform: &Matrix4x4, plane: &Plane) -> Option<CollisionData>;
}


pub struct Contact {
    pub point: Vector3,
    pub normal: Vector3,

    pub depth: f64
}

pub struct CollisionData {
    pub contacts: Vec<Contact>
}

pub struct ImpulseData {
    pub impulse: Vector3,

    pub r_a: Vector3,
    pub r_b: Vector3
}

pub struct Sphere {
    pub radius: f64
}

pub struct Cuboid {
    pub half_extents: Vector3
}
pub struct Plane {
    pub normal: Vector3,
    pub offset: f64
}
impl CollisionData {
    pub fn solve(&self, body_a: &mut RigidBody, body_b: &mut RigidBody) {
        let total_inverse_mass = body_a.inverse_mass + body_b.inverse_mass;
        let inverse_len = 1.0 / self.contacts.len() as f64;

        let mut impulses = vec![];

        for contact in  &self.contacts {
            // what the hell do I call these
            let r_a = contact.point - body_a.position;
            let r_b = contact.point - body_b.position;
            let v_a = body_a.linear_velocity + body_a.rotational_velocity.cross(&r_a);
            let v_b = body_b.linear_velocity + body_b.rotational_velocity.cross(&r_b);

            let relative_velocity = v_b - v_a;
            let velocity_along_normal = relative_velocity.dot(&contact.normal);

            if velocity_along_normal > 0.0 { return; }

            let r_a_perpendicular = r_a.cross(&contact.normal).cross(&r_a);
            let r_b_perpendicular = r_b.cross(&contact.normal).cross(&r_b);

            // ??? wtf do I call this variable
            let inverse_inertia_vector = body_a.inverse_inertia_tensor_world * r_a_perpendicular +
                body_b.inverse_inertia_tensor_world * r_b_perpendicular;

            println!("{:?} {:?}", body_a.inverse_inertia_tensor_world * r_a_perpendicular, body_b.inverse_inertia_tensor_world * r_b_perpendicular);

            let impulse_scalar = -(1.0 + RESTITUTION) * velocity_along_normal / (total_inverse_mass + inverse_inertia_vector.dot(&contact.normal));
            let impulse = contact.normal * impulse_scalar * inverse_len;

            impulses.push(ImpulseData { impulse, r_a, r_b });
        }
        for impulse_data in impulses {
            body_a.linear_velocity -= impulse_data.impulse * body_a.inverse_mass;
            body_a.rotational_velocity -= body_a.inverse_inertia_tensor_world * impulse_data.r_a.cross(&impulse_data.impulse);

            body_b.linear_velocity += impulse_data.impulse * body_b.inverse_mass;
            body_b.rotational_velocity += body_b.inverse_inertia_tensor_world * impulse_data.r_b.cross(&impulse_data.impulse);
        }
    }
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

    fn collides_with_sphere(&self, this_transform: &Matrix4x4, sphere: &Sphere, sphere_transform: &Matrix4x4) -> Option<CollisionData> {
        sphere_vs_sphere(self, this_transform, sphere, sphere_transform)
    }

    fn collides_with_cuboid(&self, this_transform: &Matrix4x4, cuboid: &Cuboid, cuboid_transform: &Matrix4x4) -> Option<CollisionData> {
        sphere_vs_cuboid(self, this_transform, cuboid, cuboid_transform)
    }


    fn collides_with_plane(&self, this_transform: &Matrix4x4, plane: &Plane) -> Option<CollisionData> {
        sphere_vs_plane(self, this_transform, plane)
    }
}

impl Collider for Cuboid {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3x3 {
        let extents = 2.0 * self.half_extents;

        let xx = extents.x * extents.x;
        let yy = extents.y * extents.y;
        let zz = extents.z * extents.z;

        let inverse_inertia_xx = 12.0 / (mass * (yy + zz));
        let inverse_inertia_yy = 12.0 / (mass * (xx + zz));
        let inverse_inertia_zz = 12.0 / (mass * (xx + yy));

        Matrix3x3 {
            m11: inverse_inertia_xx, m12: 0.0, m13: 0.0,
            m21: 0.0, m22: inverse_inertia_yy, m23: 0.0,
            m31: 0.0, m32: 0.0, m33: inverse_inertia_zz
        }
    }

    fn collides(&self, this_transform: &Matrix4x4, other: &impl Collider, other_transform: &Matrix4x4) -> Option<CollisionData> {
        other.collides_with_cuboid(other_transform, self, this_transform)
    }

    fn collides_with_sphere(&self, this_transform: &Matrix4x4, sphere: &Sphere, sphere_transform: &Matrix4x4) -> Option<CollisionData> {
        return if let Some(mut collision_data) = sphere_vs_cuboid(sphere, sphere_transform, self, this_transform) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_cuboid(&self, this_transform: &Matrix4x4, cuboid: &Cuboid, cuboid_transform: &Matrix4x4) -> Option<CollisionData> {
        cuboid_vs_cuboid(self, this_transform, cuboid, cuboid_transform)
    }

    fn collides_with_plane(&self, this_transform: &Matrix4x4, plane: &Plane) -> Option<CollisionData> {
        cuboid_vs_plane(self, this_transform, plane)
    }
}

impl Collider for Plane {
    fn calculate_inverse_inertia_tensor(&self, _mass: f64) -> Matrix3x3 {
        warn!("Plane does not have inertia tensor");

        Matrix3x3::ZERO
    }

    fn collides(&self, _this_transform: &Matrix4x4, other: &impl Collider, other_transform: &Matrix4x4) -> Option<CollisionData> {
        other.collides_with_plane(other_transform, self)
    }

    fn collides_with_sphere(&self, _this_transform: &Matrix4x4, sphere: &Sphere, sphere_transform: &Matrix4x4) -> Option<CollisionData> {
        return if let Some(mut collision_data) = sphere_vs_plane(sphere, sphere_transform, self) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_cuboid(&self, _this_transform: &Matrix4x4, cuboid: &Cuboid, cuboid_transform: &Matrix4x4) -> Option<CollisionData> {
        return if let Some(mut collision_data) = cuboid_vs_plane(cuboid, cuboid_transform, self) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_plane(&self, _this_transform: &Matrix4x4, _plane: &Plane) -> Option<CollisionData> {
        warn!("Plane vs Plane collision is not supported");

        None
    }
}

impl Cuboid {
    pub fn vertices(&self) -> [Vector3; 8] {
        [
            Vector3 { x: -self.half_extents.x, y: -self.half_extents.y, z: -self.half_extents.z },
            Vector3 { x: -self.half_extents.x, y: -self.half_extents.y, z: self.half_extents.z },
            Vector3 { x: -self.half_extents.x, y: self.half_extents.y, z: -self.half_extents.z },
            Vector3 { x: -self.half_extents.x, y: self.half_extents.y, z: self.half_extents.z },
            Vector3 { x: self.half_extents.x, y: -self.half_extents.y, z: -self.half_extents.z },
            Vector3 { x: self.half_extents.x, y: -self.half_extents.y, z: self.half_extents.z },
            Vector3 { x: self.half_extents.x, y: self.half_extents.y, z: -self.half_extents.z },
            Vector3 { x: self.half_extents.x, y: self.half_extents.y, z: self.half_extents.z }
        ]
    }
}

// fn get_separating_axes(transform_a: &Matrix4x4, transform_b: &Matrix4x4) -> [Vector3; 15] {
//     let (x_a, y_a, z_a) = transform_a.axes();
//     let (x_b, y_b, z_b) = transform_b.axes();
//
//     [
//         x_a, y_a, z_a,
//         x_b, y_b, z_b,
//         x_a.cross(&x_b),
//         x_a.cross(&y_b),
//         x_a.cross(&z_b),
//         y_a.cross(&x_b),
//         y_a.cross(&y_b),
//         y_a.cross(&z_b),
//         z_a.cross(&x_b),
//         z_a.cross(&y_b),
//         z_a.cross(&z_b)
//     ]
// }
// fn project_vertices_on_axis(vertices: &[Vector3], axis: &Vector3) -> (f64, f64) {
//     let mut min = f64::MAX;
//     let mut max = f64::MIN;
//
//     for vertex in vertices {
//         let projection = vertex.dot(axis);
//
//         if projection < min { min = projection; }
//         if projection > max { max = projection; }
//     }
//
//     (min, max)
// }

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
        depth: radius_sum - distance
    };

    Some(CollisionData { contacts: vec![contact] })
}

fn sphere_vs_cuboid(
    sphere: &Sphere,
    sphere_transform: &Matrix4x4,
    cuboid: &Cuboid,
    cuboid_transform: &Matrix4x4
) -> Option<CollisionData> {
    let center = sphere_transform.translation();
    let relative_center = cuboid_transform.inverse() * center;

    if
        relative_center.x.abs() - sphere.radius > cuboid.half_extents.x ||
        relative_center.y.abs() - sphere.radius > cuboid.half_extents.y ||
        relative_center.z.abs() - sphere.radius > cuboid.half_extents.z
    { return None; }

    let closest_point = Vector3 {
        x: relative_center.x.clamp(-cuboid.half_extents.x, cuboid.half_extents.x),
        y: relative_center.y.clamp(-cuboid.half_extents.y, cuboid.half_extents.y),
        z: relative_center.z.clamp(-cuboid.half_extents.z, cuboid.half_extents.z)
    };
    let distance_sq = (relative_center - closest_point).magnitude_sq();

    if distance_sq > sphere.radius * sphere.radius { return None; }

    let closest_point_world = *cuboid_transform * closest_point;

    let contact = Contact {
        point: closest_point_world,
        normal: (center - closest_point_world).normalized(),
        depth: sphere.radius - distance_sq.sqrt()
    };

    Some(CollisionData { contacts: vec![contact] })
}

fn sphere_vs_plane(
    sphere: &Sphere,
    sphere_transform: &Matrix4x4,
    plane: &Plane
) -> Option<CollisionData> {
    let sphere_center = sphere_transform.translation();
    let distance_to_plane = sphere_center.dot(&plane.normal) - plane.offset;

    if distance_to_plane > sphere.radius { return None; }

    let contact = Contact {
        point: sphere_center - plane.normal * (distance_to_plane - sphere.radius),
        normal: plane.normal,
        depth: sphere.radius - distance_to_plane
    };

    Some(CollisionData { contacts: vec![contact] })
}

fn cuboid_vs_cuboid(
    cuboid_a: &Cuboid,
    transform_a: &Matrix4x4,
    cuboid_b: &Cuboid,
    transform_b: &Matrix4x4
) -> Option<CollisionData> {
    warn!("Box vs Box collision is not yet complete");

    // let all_axes = get_separating_axes(transform_a, transform_b);
    //
    // let vertices_a = cuboid_a.vertices().map(|vertex| *transform_a * vertex);
    // let vertices_b = cuboid_b.vertices().map(|vertex| *transform_b * vertex);
    //
    // let mut min_overlap = f64::MAX;
    // let mut smallest_axis = Vector3::ZERO;
    //
    // for axis in all_axes {
    //     if axis == Vector3::ZERO { continue; }
    //
    //     let axis = axis.normalized();
    //
    //     let (min_a, max_a) = project_vertices_on_axis(&vertices_a, &axis);
    //     let (min_b, max_b) = project_vertices_on_axis(&vertices_b, &axis);
    //
    //     if max_a < min_b || max_b < min_a { return None; }
    //
    //     let overlap = max_a.min(max_b) - min_a.max(min_b);
    //
    //     if overlap < min_overlap {
    //         min_overlap = overlap;
    //         smallest_axis = axis;
    //     }
    // }

    None
}

fn cuboid_vs_plane(
    cuboid: &Cuboid,
    transform_cuboid: &Matrix4x4,
    plane: &Plane
) -> Option<CollisionData> {
    let mut contacts = vec![];

    for vertex in cuboid.vertices() {
        let world_point = *transform_cuboid * vertex;
        let distance_to_plane = world_point.dot(&plane.normal);

        if distance_to_plane <= plane.offset {
            contacts.push(Contact {
                point: world_point + plane.normal * (distance_to_plane - plane.offset),
                normal: plane.normal,
                depth: plane.offset - distance_to_plane
            });
        }
    }

    if contacts.is_empty() { return None; }

    Some(CollisionData { contacts })
}