use log::warn;
use nalgebra::{Matrix3, Matrix4, Point3, Vector3, zero};
use crate::dynamics::RigidBody;

pub const RESTITUTION: f64 = 0.625;

pub trait Collider {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3<f64>;
    fn collides(&self, this_transform: &Matrix4<f64>, other: &impl Collider, other_transform: &Matrix4<f64>) -> Option<CollisionData>;

    fn collides_with_sphere(&self, this_transform: &Matrix4<f64>, sphere: &Sphere, sphere_transform: &Matrix4<f64>) -> Option<CollisionData>;
    fn collides_with_cuboid(&self, this_transform: &Matrix4<f64>, cuboid: &Cuboid, cuboid_transform: &Matrix4<f64>) -> Option<CollisionData>;
    fn collides_with_plane(&self, this_transform: &Matrix4<f64>, plane: &Plane) -> Option<CollisionData>;
}

pub struct Contact {
    pub point: Point3<f64>,
    pub normal: Vector3<f64>,

    pub depth: f64
}

pub struct CollisionData {
    pub contacts: Vec<Contact>
}

pub struct Sphere {
    pub radius: f64
}

pub struct Cuboid {
    pub half_extents: Vector3<f64>
}
pub struct Plane {
    pub normal: Vector3<f64>,
    pub offset: f64
}

impl CollisionData {
    pub fn solve(&self, body_a: &mut RigidBody, body_b: &mut RigidBody) {
        let linear_component = body_a.inverse_mass + body_b.inverse_mass;
        let total_contacts = self.contacts.len() as f64;

        let mut impulses = vec![];
        let mut corrections = vec![];

        for contact in self.contacts.iter() {
            let relative_contact_a = contact.point - body_a.position;
            let relative_contact_b = contact.point - body_b.position;

            let velocity_a = body_a.linear_velocity + body_a.rotational_velocity.cross(&relative_contact_a);
            let velocity_b = body_b.linear_velocity + body_b.rotational_velocity.cross(&relative_contact_b);

            let relative_velocity = velocity_b - velocity_a;
            let contact_velocity = relative_velocity.dot(&contact.normal);

            if contact_velocity > 0.0 { return; }

            let rotation_per_unit_impulse_a = body_a.inverse_inertia_tensor_world * relative_contact_a.cross(&contact.normal);
            let rotation_per_unit_impulse_b = body_b.inverse_inertia_tensor_world * relative_contact_b.cross(&contact.normal);

            let angular_component = rotation_per_unit_impulse_a.cross(&relative_contact_a) + rotation_per_unit_impulse_b.cross(&relative_contact_b);
            let impulse_scalar = -(1.0 + RESTITUTION) * contact_velocity / (linear_component + angular_component.dot(&contact.normal));

            let impulse = contact.normal * impulse_scalar / total_contacts;
            let correction = contact.normal * (contact.depth / linear_component) / total_contacts;

            impulses.push((impulse, relative_contact_a, relative_contact_b));
            corrections.push(correction);
        }

        for (impulse, r_a, r_b) in impulses {
            if body_a.has_finite_mass() {
                body_a.linear_velocity -= impulse * body_a.inverse_mass;
                body_a.rotational_velocity -= body_a.inverse_inertia_tensor_world * r_a.cross(&impulse);
            }
            if body_b.has_finite_mass() {
                body_b.linear_velocity += impulse * body_b.inverse_mass;
                body_b.rotational_velocity += body_b.inverse_inertia_tensor_world * r_b.cross(&impulse);
            }
        }
        for correction in corrections {
            if body_a.has_finite_mass() {
                body_a.position -= correction * body_a.inverse_mass;
            }
            if body_b.has_finite_mass() {
                body_b.position += correction * body_b.inverse_mass;
            }
        }
    }
}

impl Collider for Sphere {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3<f64> {
        let inertia = 2.0 * mass * self.radius * self.radius / 5.0;
        let inertia_tensor = Matrix3::from_diagonal_element(inertia);

        inertia_tensor.try_inverse().unwrap_or(zero())
    }
    fn collides(&self, this_transform: &Matrix4<f64>, other: &impl Collider, other_transform: &Matrix4<f64>) -> Option<CollisionData> {
        other.collides_with_sphere(other_transform, self, this_transform)
    }

    fn collides_with_sphere(&self, this_transform: &Matrix4<f64>, sphere: &Sphere, sphere_transform: &Matrix4<f64>) -> Option<CollisionData> {
        sphere_vs_sphere(self, this_transform, sphere, sphere_transform)
    }

    fn collides_with_cuboid(&self, this_transform: &Matrix4<f64>, cuboid: &Cuboid, cuboid_transform: &Matrix4<f64>) -> Option<CollisionData> {
        sphere_vs_cuboid(self, this_transform, cuboid, cuboid_transform)
    }


    fn collides_with_plane(&self, this_transform: &Matrix4<f64>, plane: &Plane) -> Option<CollisionData> {
        sphere_vs_plane(self, this_transform, plane)
    }
}

impl Collider for Cuboid {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3<f64> {
        let extents = 2.0 * self.half_extents;

        let xx = extents.x * extents.x;
        let yy = extents.y * extents.y;
        let zz = extents.z * extents.z;

        let inertia_diagonal = mass * Vector3::new(yy + zz, xx + zz, xx + yy) / 12.0;
        let inertia_tensor = Matrix3::from_diagonal(&inertia_diagonal);

        inertia_tensor.try_inverse().unwrap_or(zero())
    }

    fn collides(&self, this_transform: &Matrix4<f64>, other: &impl Collider, other_transform: &Matrix4<f64>) -> Option<CollisionData> {
        other.collides_with_cuboid(other_transform, self, this_transform)
    }

    fn collides_with_sphere(&self, this_transform: &Matrix4<f64>, sphere: &Sphere, sphere_transform: &Matrix4<f64>) -> Option<CollisionData> {
        return if let Some(mut collision_data) = sphere_vs_cuboid(sphere, sphere_transform, self, this_transform) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_cuboid(&self, _this_transform: &Matrix4<f64>, _cuboid: &Cuboid, _cuboid_transform: &Matrix4<f64>) -> Option<CollisionData> {
        warn!("Cuboid vs Cuboid collision is not supported");

        None
    }

    fn collides_with_plane(&self, this_transform: &Matrix4<f64>, plane: &Plane) -> Option<CollisionData> {
        cuboid_vs_plane(self, this_transform, plane)
    }
}

impl Collider for Plane {
    fn calculate_inverse_inertia_tensor(&self, _mass: f64) -> Matrix3<f64> {
        warn!("Plane does not have inertia tensor");

        zero()
    }

    fn collides(&self, _this_transform: &Matrix4<f64>, other: &impl Collider, other_transform: &Matrix4<f64>) -> Option<CollisionData> {
        other.collides_with_plane(other_transform, self)
    }

    fn collides_with_sphere(&self, _this_transform: &Matrix4<f64>, sphere: &Sphere, sphere_transform: &Matrix4<f64>) -> Option<CollisionData> {
        return if let Some(mut collision_data) = sphere_vs_plane(sphere, sphere_transform, self) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_cuboid(&self, _this_transform: &Matrix4<f64>, cuboid: &Cuboid, cuboid_transform: &Matrix4<f64>) -> Option<CollisionData> {
        return if let Some(mut collision_data) = cuboid_vs_plane(cuboid, cuboid_transform, self) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_plane(&self, _this_transform: &Matrix4<f64>, _plane: &Plane) -> Option<CollisionData> {
        warn!("Plane vs Plane collision is not supported");

        None
    }
}

impl Cuboid {
    pub fn vertices(&self) -> [Point3<f64>; 8] {
        [
            Point3::new(-self.half_extents.x, -self.half_extents.y, -self.half_extents.z),
            Point3::new(-self.half_extents.x, -self.half_extents.y, self.half_extents.z),
            Point3::new(-self.half_extents.x, self.half_extents.y, -self.half_extents.z),
            Point3::new(-self.half_extents.x, self.half_extents.y, self.half_extents.z),
            Point3::new(self.half_extents.x, -self.half_extents.y, -self.half_extents.z),
            Point3::new(self.half_extents.x, -self.half_extents.y, self.half_extents.z),
            Point3::new(self.half_extents.x, self.half_extents.y, -self.half_extents.z),
            Point3::new(self.half_extents.x, self.half_extents.y, self.half_extents.z)
        ]
    }
}

fn get_translation(transform: &Matrix4<f64>) -> Point3<f64> {
    Point3::new(transform.m14, transform.m24, transform.m34)
}

fn sphere_vs_sphere(
    sphere_a: &Sphere,
    transform_a: &Matrix4<f64>,
    sphere_b: &Sphere,
    transform_b: &Matrix4<f64>
) -> Option<CollisionData> {
    let center_a = get_translation(transform_a);
    let center_b = get_translation(transform_b);

    let delta = center_a - center_b;
    let distance_sq = delta.magnitude_squared();
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
    sphere_transform: &Matrix4<f64>,
    cuboid: &Cuboid,
    cuboid_transform: &Matrix4<f64>
) -> Option<CollisionData> {
    let center = get_translation(sphere_transform);
    let relative_center = cuboid_transform.try_inverse()?.transform_point(&center);

    if
        relative_center.x.abs() - sphere.radius > cuboid.half_extents.x ||
        relative_center.y.abs() - sphere.radius > cuboid.half_extents.y ||
        relative_center.z.abs() - sphere.radius > cuboid.half_extents.z
    { return None; }

    let closest_point = Point3::new(
        relative_center.x.clamp(-cuboid.half_extents.x, cuboid.half_extents.x),
        relative_center.y.clamp(-cuboid.half_extents.y, cuboid.half_extents.y),
        relative_center.z.clamp(-cuboid.half_extents.z, cuboid.half_extents.z)
    );
    let distance_sq = (relative_center - closest_point).magnitude_squared();

    if distance_sq > sphere.radius * sphere.radius { return None; }

    let closest_point_world = cuboid_transform.transform_point(&closest_point);

    let contact = Contact {
        point: closest_point_world,
        normal: (center - closest_point_world).normalize(),
        depth: sphere.radius - distance_sq.sqrt()
    };

    Some(CollisionData { contacts: vec![contact] })
}

fn sphere_vs_plane(
    sphere: &Sphere,
    sphere_transform: &Matrix4<f64>,
    plane: &Plane
) -> Option<CollisionData> {
    let sphere_center = get_translation(sphere_transform);
    let distance_to_plane = sphere_center.coords.dot(&plane.normal) - plane.offset;

    if distance_to_plane > sphere.radius { return None; }

    let contact = Contact {
        point: sphere_center - plane.normal * distance_to_plane,
        normal: plane.normal,
        depth: sphere.radius - distance_to_plane
    };

    Some(CollisionData { contacts: vec![contact] })
}

fn cuboid_vs_plane(
    cuboid: &Cuboid,
    transform_cuboid: &Matrix4<f64>,
    plane: &Plane
) -> Option<CollisionData> {
    let mut contacts = vec![];

    for vertex in cuboid.vertices() {
        let world_point = transform_cuboid.transform_point(&vertex);
        let distance_to_plane = world_point.coords.dot(&plane.normal);

        if distance_to_plane <= plane.offset {
            let separation = plane.offset - distance_to_plane;

            contacts.push(Contact {
                point: world_point + plane.normal * separation,
                normal: plane.normal,
                depth: separation
            });
        }
    }

    if contacts.is_empty() { return None; }

    Some(CollisionData { contacts })
}