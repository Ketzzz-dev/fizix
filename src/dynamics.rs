use crate::collisions::{Collider, ResolutionInfo};
use nalgebra::{zero, Matrix3, Matrix4, Point3, Quaternion, Vector3};
use std::ops::Deref;
use three_d::Zero;

pub struct World {
    pub bodies: Vec<RigidBody>,
    pub colliders: Vec<Box<dyn Collider>>,

    pub gravity: Vector3<f64>
}
impl World {
    pub fn new() -> Self {
        Self {
            bodies: vec![],
            colliders: vec![],

            gravity: Vector3::new(0.0, 9.81, 0.0)
        }
    }

    pub fn create_dynamic_body(
        &mut self,
        settings: DynamicRigidBodySettings,
        collider: impl Collider + 'static
    ) -> usize {
        let index = self.bodies.len();

        self.bodies.push(RigidBody::dynamic_body(settings, &collider));
        self.colliders.push(Box::new(collider));

        index
    }
    pub fn create_static_body(
        &mut self,
        settings: StaticRigidBodySettings,
        collider: impl Collider + 'static
    ) -> usize {
        let index = self.bodies.len();

        self.bodies.push(RigidBody::static_body(settings));
        self.colliders.push(Box::new(collider));

        index
    }
    pub fn get_body(&self, index: usize) -> Option<&RigidBody> {
        self.bodies.get(index)
    }
    pub fn get_body_mut(&mut self, index: usize) -> Option<&mut RigidBody> {
        self.bodies.get_mut(index)
    }
    pub fn get_collider(&self, index: usize) -> Option<&Box<dyn Collider>> {
        self.colliders.get(index)
    }
    pub fn get_collider_mut(&mut self, index: usize) -> Option<&mut Box<dyn Collider>> {
        self.colliders.get_mut(index)
    }

    pub fn update(&mut self, delta_time: f64) {
        let mut collisions = vec![];
        let len = self.bodies.len();

        for a in 0..len {
            for b in a + 1..len {
                let (body_a, body_b) = (&self.bodies[a], &self.bodies[b]);

                if body_a.has_finite_mass() || body_b.has_finite_mass() {
                    let (collider_a, collider_b) = (&self.colliders[a], &self.colliders[b]);

                    if let Some(collision_data) = collider_a.collides(
                        &body_a.transform,
                        collider_b.deref(),
                        &body_b.transform
                    ) {
                        collisions.push((collision_data, a, b));
                    }
                }
            }
        }
        for (collision_data, a, b) in collisions {
            let (body_a, body_b) = (&self.bodies[a], &self.bodies[b]);

            if let Some(ResolutionInfo {
                linear_impulse,
                angular_impulse_a,
                angular_impulse_b,
                correction
            }) = collision_data.solve(body_a, body_b) {
                let body_a = &mut self.bodies[a];

                if body_a.has_finite_mass() {
                    body_a.linear_velocity -= body_a.inverse_mass * linear_impulse;
                    body_a.angular_velocity -= body_a.inverse_inertia_tensor_world * angular_impulse_a;
                    body_a.position -= body_a.inverse_mass * correction;
                }

                let body_b = &mut self.bodies[b];

                if body_b.has_finite_mass() {
                    body_b.linear_velocity += body_b.inverse_mass * linear_impulse;
                    body_b.angular_velocity += body_b.inverse_inertia_tensor_world * angular_impulse_b;
                    body_b.position += body_b.inverse_mass * correction;
                }
            }
        }
        for body in self.bodies.iter_mut() {
            if body.has_finite_mass() {
                body.force -= self.gravity / body.inverse_mass;

                body.update(delta_time);
            }
        }
    }
}

pub struct RigidBody {
    pub position: Point3<f64>,
    pub orientation: Quaternion<f64>,

    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,

    pub linear_damping: f64,
    pub angular_damping: f64,

    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,

    pub inverse_mass: f64,

    pub inverse_inertia_tensor_local: Matrix3<f64>,
    pub inverse_inertia_tensor_world: Matrix3<f64>,

    pub transform: Matrix4<f64>
}

impl RigidBody {
    pub fn dynamic_body(settings: DynamicRigidBodySettings, collider: &impl Collider) -> Self {
        assert!(settings.mass > 0.0, "Mass must be greater than zero");

        let mut rigid_body = Self {
            position: settings.position,
            orientation: settings.orientation,

            linear_velocity: settings.linear_velocity,
            angular_velocity: settings.angular_velocity,

            linear_damping: settings.linear_damping.clamp(0.0, 1.0),
            angular_damping: settings.angular_damping.clamp(0.0, 1.0),

            force: zero(),
            torque: zero(),

            inverse_mass: 1.0 / settings.mass,

            inverse_inertia_tensor_local: collider.calculate_inverse_inertia_tensor(settings.mass),
            inverse_inertia_tensor_world: zero(),

            transform: Matrix4::identity()
        };

        rigid_body.calculate_transform();
        rigid_body.calculate_inertia_tensor();

        rigid_body
    }
    pub fn static_body(settings: StaticRigidBodySettings) -> Self {
        let mut rigid_body = Self {
            position: settings.position,
            orientation: settings.orientation,

            linear_velocity: zero(),
            angular_velocity: zero(),

            linear_damping: 0.0,
            angular_damping: 0.0,

            force: zero(),
            torque: zero(),

            inverse_mass: 0.0,

            inverse_inertia_tensor_local: zero(),
            inverse_inertia_tensor_world: zero(),

            transform: Matrix4::identity()
        };

        rigid_body.calculate_transform();

        rigid_body
    }

    pub fn has_finite_mass(&self) -> bool {
        self.inverse_mass > 0.0
    }

    pub fn update(&mut self, delta_time: f64) {
        if self.has_finite_mass() {
            self.integrate(delta_time);
            self.calculate_transform();
            self.calculate_inertia_tensor();
        }
    }

    // leapfrog integration
    fn integrate(&mut self, delta_time: f64) {
        let linear_acceleration = self.inverse_mass * self.force;
        let angular_acceleration = self.inverse_inertia_tensor_world * self.torque;
        let half_delta_time = 0.5 * delta_time;

        // half-step velocities
        self.linear_velocity += linear_acceleration * half_delta_time;
        self.angular_velocity += angular_acceleration * half_delta_time;

        // full step positions
        self.position += self.linear_velocity * delta_time;
        self.orientation +=
            Quaternion::from_imag(0.5 * self.angular_velocity) * self.orientation * delta_time;

        self.orientation.normalize_mut();

        // 2nd half-step velocities (full step)
        self.linear_velocity += linear_acceleration * half_delta_time;
        self.angular_velocity += angular_acceleration * half_delta_time;

        // Impose drag
        self.linear_velocity *= self.linear_damping.powf(delta_time);
        self.angular_velocity *= self.angular_damping.powf(delta_time);

        // reset forces
        self.force.set_zero();
        self.torque.set_zero();
    }

    fn calculate_transform(&mut self) {
        // Quaternion to rotation matrix
        let jj = self.orientation.j * self.orientation.j;
        let kk = self.orientation.k * self.orientation.k;
        let ij = self.orientation.i * self.orientation.j;
        let wk = self.orientation.w * self.orientation.k;
        let wj = self.orientation.w * self.orientation.j;
        let jk = self.orientation.j * self.orientation.k;
        let wi = self.orientation.w * self.orientation.i;
        let ii = self.orientation.i * self.orientation.i;
        let ik = self.orientation.i * self.orientation.k;

        self.transform.m11 = 1.0 - 2.0 * (jj + kk);
        self.transform.m12 = 2.0 * (ij - wk);
        self.transform.m13 = 2.0 * (ik + wj);
        self.transform.m14 = self.position.x;

        self.transform.m21 = 2.0 * (ij + wk);
        self.transform.m22 = 1.0 - 2.0 * (ii + kk);
        self.transform.m23 = 2.0 * (jk - wi);
        self.transform.m24 = self.position.y;

        self.transform.m31 = 2.0 * (ik - wj);
        self.transform.m32 = 2.0 * (jk + wi);
        self.transform.m33 = 1.0 - 2.0 * (ii + jj);
        self.transform.m34 = self.position.z;
    }

    fn calculate_inertia_tensor(&mut self) {
        let rotation_matrix = Matrix3::new(
            self.transform.m11,
            self.transform.m12,
            self.transform.m13,
            self.transform.m21,
            self.transform.m22,
            self.transform.m23,
            self.transform.m31,
            self.transform.m32,
            self.transform.m33,
        );

        // world_tensor = R * local_tensor * R^T
        self.inverse_inertia_tensor_world =
            rotation_matrix * self.inverse_inertia_tensor_local * rotation_matrix.transpose();
    }
}

pub struct DynamicRigidBodySettings {
    pub position: Point3<f64>,
    pub orientation: Quaternion<f64>,

    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,

    pub linear_damping: f64,
    pub angular_damping: f64,

    pub mass: f64,
}

impl Default for DynamicRigidBodySettings {
    fn default() -> Self {
        Self {
            position: Point3::origin(),
            orientation: Quaternion::identity(),

            linear_damping: 0.9,
            angular_damping: 0.9,

            linear_velocity: zero(),
            angular_velocity: zero(),

            mass: 1.0,
        }
    }
}

pub struct StaticRigidBodySettings {
    pub position: Point3<f64>,
    pub orientation: Quaternion<f64>,
}

impl Default for StaticRigidBodySettings {
    fn default() -> Self {
        Self {
            position: Point3::origin(),
            orientation: Quaternion::identity(),
        }
    }
}
