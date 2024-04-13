use crate::dynamics::RigidBody;
use nalgebra::{zero, Matrix3, Matrix4, Point3, Vector3};
use three_d::SquareMatrix;

pub const RESTITUTION: f64 = 0.4;
pub const STATIC_FRICTION: f64 = 0.5;
pub const DYNAMIC_FRICTION: f64 = 0.3;
pub trait Collider {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3<f64>;
    fn calculate_scale_matrix(&self) -> three_d::Matrix4<f32>;

    fn collides(
        &self,
        self_transform: &Matrix4<f64>,
        other: &dyn Collider,
        other_transform: &Matrix4<f64>,
    ) -> Option<CollisionData>;

    fn collides_with_sphere(
        &self,
        self_transform: &Matrix4<f64>,
        sphere: &Sphere,
        sphere_transform: &Matrix4<f64>,
    ) -> Option<CollisionData>;
    fn collides_with_cuboid(
        &self,
        self_transform: &Matrix4<f64>,
        cuboid: &Cuboid,
        cuboid_transform: &Matrix4<f64>,
    ) -> Option<CollisionData>;
    fn collides_with_plane(
        &self,
        self_transform: &Matrix4<f64>,
        plane: &Plane,
    ) -> Option<CollisionData>;
}

pub struct Contact {
    pub point: Point3<f64>,
    pub normal: Vector3<f64>,

    pub depth: f64,
}

pub struct ResolutionInfo {
    pub linear_impulse: Vector3<f64>,
    pub angular_impulse_a: Vector3<f64>,
    pub angular_impulse_b: Vector3<f64>,
    pub correction: Vector3<f64>
}

pub struct CollisionData {
    pub contacts: Vec<Contact>,
}

impl CollisionData {
    // This is a temporary collision resolution method
    pub fn solve(&self, body_a: &RigidBody, body_b: &RigidBody) -> Option<ResolutionInfo> {
        let linear_component = body_a.inverse_mass + body_b.inverse_mass;

        let mut linear_impulse = zero::<Vector3<f64>>();
        let mut angular_impulse_a = zero::<Vector3<f64>>();
        let mut angular_impulse_b = zero::<Vector3<f64>>();
        let mut correction = zero::<Vector3<f64>>();

        for contact in self.contacts.iter() {
            let relative_contact_a = contact.point - body_a.position;
            let relative_contact_b = contact.point - body_b.position;

            let velocity_a =
                body_a.linear_velocity + body_a.angular_velocity.cross(&relative_contact_a);
            let velocity_b =
                body_b.linear_velocity + body_b.angular_velocity.cross(&relative_contact_b);

            let relative_velocity = velocity_b - velocity_a;
            let contact_velocity = relative_velocity.dot(&contact.normal);

            if contact_velocity > 0.0 {
                return None
            }

            let rotation_per_unit_impulse_a =
                body_a.inverse_inertia_tensor_world * relative_contact_a.cross(&contact.normal);
            let rotation_per_unit_impulse_b =
                body_b.inverse_inertia_tensor_world * relative_contact_b.cross(&contact.normal);

            let velocity_per_unit_impulse_a =
                rotation_per_unit_impulse_a.cross(&relative_contact_a);
            let velocity_per_unit_impulse_b =
                rotation_per_unit_impulse_b.cross(&relative_contact_b);

            let angular_component =
                (velocity_per_unit_impulse_a + velocity_per_unit_impulse_b).dot(&contact.normal);
            let denominator = linear_component + angular_component;

            let reaction_scalar = -(1.0 + RESTITUTION) * contact_velocity / denominator;
            let reaction_impulse = contact.normal * reaction_scalar;

            linear_impulse += reaction_impulse;
            angular_impulse_a += relative_contact_a.cross(&reaction_impulse);
            angular_impulse_b += relative_contact_b.cross(&reaction_impulse);

            let mut contact_tangent = relative_velocity - contact.normal * contact_velocity;

            if contact_tangent.magnitude_squared() == 0.0 {
                continue;
            }

            contact_tangent.normalize_mut();

            let friction_scalar = -relative_velocity.dot(&contact_tangent) / denominator;
            let friction_impulse = if friction_scalar.abs() < reaction_scalar * STATIC_FRICTION {
                contact_tangent * friction_scalar
            } else {
                contact_tangent * -reaction_scalar * DYNAMIC_FRICTION
            };

            linear_impulse += friction_impulse;
            angular_impulse_a += relative_contact_a.cross(&friction_impulse);
            angular_impulse_b += relative_contact_b.cross(&friction_impulse);

            correction += contact.normal * (contact.depth / linear_component);
        }

        let inverse_contact_length = 1.0 / self.contacts.len() as f64;

        linear_impulse *= inverse_contact_length;
        angular_impulse_a *= inverse_contact_length;
        angular_impulse_b *= inverse_contact_length;
        correction *= inverse_contact_length;

        Some(ResolutionInfo {
            linear_impulse,
            angular_impulse_a,
            angular_impulse_b,
            correction
        })
    }
}

pub struct Sphere {
    pub radius: f64,
}

impl Collider for Sphere {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3<f64> {
        // I = 2m * r^2 / 5
        let inertia = 2.0 * mass * self.radius * self.radius / 5.0;
        let inertia_tensor = Matrix3::from_diagonal_element(inertia);

        inertia_tensor.try_inverse().unwrap_or(zero())
    }
    fn calculate_scale_matrix(&self) -> three_d::Matrix4<f32> {
        three_d::Matrix4::from_scale(self.radius as f32)
    }

    fn collides(
        &self,
        self_transform: &Matrix4<f64>,
        other: &dyn Collider,
        other_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        other.collides_with_sphere(other_transform, self, self_transform) // dynamic dispatch
    }

    fn collides_with_sphere(
        &self,
        self_transform: &Matrix4<f64>,
        sphere: &Sphere,
        sphere_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        sphere_vs_sphere(self, self_transform, sphere, sphere_transform)
    }

    fn collides_with_cuboid(
        &self,
        self_transform: &Matrix4<f64>,
        cuboid: &Cuboid,
        cuboid_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        sphere_vs_cuboid(self, self_transform, cuboid, cuboid_transform)
    }

    fn collides_with_plane(
        &self,
        self_transform: &Matrix4<f64>,
        plane: &Plane,
    ) -> Option<CollisionData> {
        sphere_vs_plane(self, self_transform, plane)
    }
}

pub struct Cuboid {
    pub half_extents: Vector3<f64>,
    pub local_vertices: [Point3<f64>; 8]
}

impl Cuboid {
    pub fn new(half_extents: Vector3<f64>) -> Self {
        Self {
            half_extents,
            local_vertices: [
                Point3::new(-half_extents.x, -half_extents.y, -half_extents.z),
                Point3::new(-half_extents.x, -half_extents.y, half_extents.z),
                Point3::new(-half_extents.x, half_extents.y, -half_extents.z),
                Point3::new(-half_extents.x, half_extents.y, half_extents.z),
                Point3::new(half_extents.x, -half_extents.y, -half_extents.z),
                Point3::new(half_extents.x, -half_extents.y, half_extents.z),
                Point3::new(half_extents.x, half_extents.y, -half_extents.z),
                Point3::new(half_extents.x, half_extents.y, half_extents.z)
            ]
        }
    }
}
impl Collider for Cuboid {
    fn calculate_inverse_inertia_tensor(&self, mass: f64) -> Matrix3<f64> {
        let extents = 2.0 * self.half_extents;

        let xx = extents.x * extents.x;
        let yy = extents.y * extents.y;
        let zz = extents.z * extents.z;

        // Ix = m * (y^2 + z^2) / 12
        // Iy = m * (x^2 + z^2) / 12
        // Iz = m * (x^2 + y^2) / 12
        let inertia_diagonal = mass * Vector3::new(yy + zz, xx + zz, xx + yy) / 12.0;
        let inertia_tensor = Matrix3::from_diagonal(&inertia_diagonal);

        inertia_tensor.try_inverse().unwrap_or(zero())
    }
    fn calculate_scale_matrix(&self) -> three_d::Matrix4<f32> {
        three_d::Matrix4::from_nonuniform_scale(
            self.half_extents.x as f32,
            self.half_extents.y as f32,
            self.half_extents.z as f32
        )
    }

    fn collides(
        &self,
        self_transform: &Matrix4<f64>,
        other: &dyn Collider,
        other_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        other.collides_with_cuboid(other_transform, self, self_transform)
    }

    fn collides_with_sphere(
        &self,
        self_transform: &Matrix4<f64>,
        sphere: &Sphere,
        sphere_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        // Invert the collision data to make it look like the sphere is colliding with the cuboid
        if let Some(mut collision_data) =
            sphere_vs_cuboid(sphere, sphere_transform, self, self_transform)
        {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_cuboid(
        &self,
        self_transform: &Matrix4<f64>,
        cuboid: &Cuboid,
        cuboid_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        cuboid_vs_cuboid(self, self_transform, cuboid, cuboid_transform)
    }

    fn collides_with_plane(
        &self,
        self_transform: &Matrix4<f64>,
        plane: &Plane,
    ) -> Option<CollisionData> {
        cuboid_vs_plane(self, self_transform, plane)
    }
}

pub struct Plane {
    pub normal: Vector3<f64>,
    pub offset: f64,
}

impl Collider for Plane {
    fn calculate_inverse_inertia_tensor(&self, _mass: f64) -> Matrix3<f64> {
        // Planes have no inertia tensor
        zero()
    }
    fn calculate_scale_matrix(&self) -> three_d::Matrix4<f32> {
        three_d::Matrix4::identity()
    }

    fn collides(
        &self,
        _self_transform: &Matrix4<f64>,
        other: &dyn Collider,
        other_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        other.collides_with_plane(other_transform, self)
    }

    fn collides_with_sphere(
        &self,
        _self_transform: &Matrix4<f64>,
        sphere: &Sphere,
        sphere_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        if let Some(mut collision_data) = sphere_vs_plane(sphere, sphere_transform, self) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_cuboid(
        &self,
        _self_transform: &Matrix4<f64>,
        cuboid: &Cuboid,
        cuboid_transform: &Matrix4<f64>,
    ) -> Option<CollisionData> {
        if let Some(mut collision_data) = cuboid_vs_plane(cuboid, cuboid_transform, self) {
            for contact in &mut collision_data.contacts {
                contact.normal = -contact.normal;
            }

            Some(collision_data)
        } else {
            None
        }
    }

    fn collides_with_plane(
        &self,
        _self_transform: &Matrix4<f64>,
        _plane: &Plane,
    ) -> Option<CollisionData> {
        // Planes cannot collide with other planes
        None
    }
}

fn get_translation(transform: &Matrix4<f64>) -> Point3<f64> {
    Point3::new(transform.m14, transform.m24, transform.m34)
}

fn sphere_vs_sphere(
    sphere_a: &Sphere,
    transform_a: &Matrix4<f64>,
    sphere_b: &Sphere,
    transform_b: &Matrix4<f64>,
) -> Option<CollisionData> {
    let center_a = get_translation(transform_a);
    let center_b = get_translation(transform_b);

    let delta = center_a - center_b;
    let distance_sq = delta.magnitude_squared();
    let radius_sum = sphere_a.radius + sphere_b.radius;

    if distance_sq > radius_sum * radius_sum {
        return None;
    }

    // Normalizing manually since distance_sq is already calculated
    let distance = distance_sq.sqrt();
    let normal = delta * (1.0 / distance);

    let contact = Contact {
        point: center_a + normal * sphere_a.radius,
        normal,
        depth: radius_sum - distance,
    };

    Some(CollisionData {
        contacts: vec![contact],
    })
}
fn sphere_vs_cuboid(
    sphere: &Sphere,
    sphere_transform: &Matrix4<f64>,
    cuboid: &Cuboid,
    cuboid_transform: &Matrix4<f64>,
) -> Option<CollisionData> {
    let center = get_translation(sphere_transform);
    let relative_center = cuboid_transform.try_inverse()?.transform_point(&center); // easier to calculate in local space

    // Early-out
    if relative_center.x.abs() - sphere.radius > cuboid.half_extents.x
        || relative_center.y.abs() - sphere.radius > cuboid.half_extents.y
        || relative_center.z.abs() - sphere.radius > cuboid.half_extents.z
    {
        return None;
    }

    let closest_point = Point3::new(
        relative_center
            .x
            .clamp(-cuboid.half_extents.x, cuboid.half_extents.x),
        relative_center
            .y
            .clamp(-cuboid.half_extents.y, cuboid.half_extents.y),
        relative_center
            .z
            .clamp(-cuboid.half_extents.z, cuboid.half_extents.z),
    );
    let distance_sq = (relative_center - closest_point).magnitude_squared();

    if distance_sq > sphere.radius * sphere.radius {
        return None;
    }

    let closest_point_world = cuboid_transform.transform_point(&closest_point); // convert back to world space

    let contact = Contact {
        point: closest_point_world,
        normal: (center - closest_point_world).normalize(),
        depth: sphere.radius - distance_sq.sqrt(),
    };

    Some(CollisionData {
        contacts: vec![contact],
    })
}
fn sphere_vs_plane(
    sphere: &Sphere,
    sphere_transform: &Matrix4<f64>,
    plane: &Plane,
) -> Option<CollisionData> {
    let sphere_center = get_translation(sphere_transform);
    let distance_to_plane = sphere_center.coords.dot(&plane.normal) - plane.offset;

    if distance_to_plane > sphere.radius {
        return None;
    }

    let contact = Contact {
        point: sphere_center - plane.normal * distance_to_plane,
        normal: plane.normal,
        depth: sphere.radius - distance_to_plane,
    };

    Some(CollisionData {
        contacts: vec![contact],
    })
}
fn cuboid_vs_plane(
    cuboid: &Cuboid,
    transform_cuboid: &Matrix4<f64>,
    plane: &Plane,
) -> Option<CollisionData> {
    let mut contacts = vec![];

    for vertex in cuboid.local_vertices.iter() {
        let world_point = transform_cuboid.transform_point(&vertex);
        let distance_to_plane = world_point.coords.dot(&plane.normal);

        if distance_to_plane <= plane.offset {
            let separation = plane.offset - distance_to_plane;

            contacts.push(Contact {
                point: world_point + plane.normal * separation,
                normal: plane.normal,
                depth: separation,
            });
        }
    }

    if contacts.is_empty() {
        return None;
    }

    Some(CollisionData { contacts })
}
fn cuboid_vs_cuboid(
    _cuboid_a: &Cuboid,
    _transform_a: &Matrix4<f64>,
    _cuboid_b: &Cuboid,
    _transform_b: &Matrix4<f64>,
) -> Option<CollisionData> {
    todo!("Implement cuboid vs cuboid collision detection")
}
