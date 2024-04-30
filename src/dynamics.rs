use crate::aliases::{Point3, Quaternion, RotationMatrix, TransformMatrix, Vector3};
use crate::collisions::Collider;
use crate::collisions::narrow_phase::{CollisionManifold, test_collision};
use crate::dynamics::solvers::resolve_collision;

pub mod solvers;

pub struct RigidBody {
    pub position: Point3,
    pub orientation: Quaternion,

    pub linear_velocity: Vector3,
    pub angular_velocity: Vector3,

    pub linear_damping: f64,
    pub angular_damping: f64,

    pub force: Vector3,
    pub torque: Vector3,

    pub inverse_mass: f64,

    pub inverse_inertia_tensor_local: RotationMatrix,
    pub inverse_inertia_tensor_world: RotationMatrix,

    pub transform_matrix: TransformMatrix
}
impl RigidBody {
    pub fn dynamic_body(settings: DynamicRigidBodySettings, collider: &Collider) -> Self {
        assert!(settings.mass > 0.0, "Mass must be greater than zero");

        let mut rigid_body = Self {
            position: settings.position,
            orientation: settings.orientation,

            linear_velocity: settings.linear_velocity,
            angular_velocity: settings.angular_velocity,

            linear_damping: settings.linear_damping.clamp(0.0, 1.0),
            angular_damping: settings.angular_damping.clamp(0.0, 1.0),

            force: Vector3::zeros(),
            torque: Vector3::zeros(),

            inverse_mass: 1.0 / settings.mass,

            inverse_inertia_tensor_local: collider.inverse_inertia_tensor(settings.mass),
            inverse_inertia_tensor_world: RotationMatrix::zeros(),

            transform_matrix: TransformMatrix::identity()
        };

        rigid_body.calculate_transform();
        rigid_body.calculate_inertia_tensor();

        rigid_body
    }
    pub fn static_body(settings: StaticRigidBodySettings) -> Self {
        let mut rigid_body = Self {
            position: settings.position,
            orientation: settings.orientation,

            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),

            linear_damping: 0.0,
            angular_damping: 0.0,

            force: Vector3::zeros(),
            torque: Vector3::zeros(),

            inverse_mass: 0.0,

            inverse_inertia_tensor_local: RotationMatrix::zeros(),
            inverse_inertia_tensor_world: RotationMatrix::zeros(),

            transform_matrix: TransformMatrix::identity()
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
        self.orientation += Quaternion::from_imag(0.5 * self.angular_velocity)
            * self.orientation * delta_time;

        self.orientation.normalize_mut();

        // 2nd half-step velocities (full step)
        self.linear_velocity += linear_acceleration * half_delta_time;
        self.angular_velocity += angular_acceleration * half_delta_time;

        // Impose drag
        self.linear_velocity *= self.linear_damping.powf(delta_time);
        self.angular_velocity *= self.angular_damping.powf(delta_time);

        // reset forces
        self.force = Vector3::zeros();
        self.torque = Vector3::zeros();
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

        self.transform_matrix.m11 = 1.0 - 2.0 * (jj + kk);
        self.transform_matrix.m12 = 2.0 * (ij - wk);
        self.transform_matrix.m13 = 2.0 * (ik + wj);
        self.transform_matrix.m14 = self.position.x;

        self.transform_matrix.m21 = 2.0 * (ij + wk);
        self.transform_matrix.m22 = 1.0 - 2.0 * (ii + kk);
        self.transform_matrix.m23 = 2.0 * (jk - wi);
        self.transform_matrix.m24 = self.position.y;

        self.transform_matrix.m31 = 2.0 * (ik - wj);
        self.transform_matrix.m32 = 2.0 * (jk + wi);
        self.transform_matrix.m33 = 1.0 - 2.0 * (ii + jj);
        self.transform_matrix.m34 = self.position.z;
    }

    fn calculate_inertia_tensor(&mut self) {
        let rotation_matrix = self.transform_matrix.fixed_view::<3, 3>(0, 0);

        // world_tensor = R * local_tensor * R^T
        self.inverse_inertia_tensor_world =
            rotation_matrix * self.inverse_inertia_tensor_local * rotation_matrix.transpose();
    }
}

pub struct DynamicRigidBodySettings {
    pub position: Point3,
    pub orientation: Quaternion,

    pub linear_velocity: Vector3,
    pub angular_velocity: Vector3,

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

            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),

            mass: 1.0,
        }
    }
}

pub struct StaticRigidBodySettings {
    pub position: Point3,
    pub orientation: Quaternion,
}
impl Default for StaticRigidBodySettings {
    fn default() -> Self {
        Self {
            position: Point3::origin(),
            orientation: Quaternion::identity(),
        }
    }
}

pub struct World {
    pub bodies: Vec<RigidBody>,
    pub colliders: Vec<Collider>,
    pub collisions: Vec<(CollisionManifold, usize, usize)>,

    pub gravity: Vector3
}
impl World {
    pub fn new() -> Self {
        Self {
            bodies: vec![],
            colliders: vec![],
            collisions: vec![],

            gravity: Vector3::new(0.0, 9.81, 0.0)
        }
    }

    pub fn create_dynamic_body(
        &mut self,
        settings: DynamicRigidBodySettings,
        collider: Collider
    ) -> usize {
        let index = self.bodies.len();

        self.bodies.push(RigidBody::dynamic_body(settings, &collider));
        self.colliders.push(collider);

        index
    }
    pub fn create_static_body(
        &mut self,
        settings: StaticRigidBodySettings,
        collider: Collider
    ) -> usize {
        let index = self.bodies.len();

        self.bodies.push(RigidBody::static_body(settings));
        self.colliders.push(collider);

        index
    }

    pub fn get_body(&self, index: usize) -> Option<&RigidBody> {
        self.bodies.get(index)
    }
    pub fn get_body_mut(&mut self, index: usize) -> Option<&mut RigidBody> {
        self.bodies.get_mut(index)
    }
    pub fn get_collider(&self, index: usize) -> Option<&Collider> {
        self.colliders.get(index)
    }
    pub fn get_collider_mut(&mut self, index: usize) -> Option<&mut Collider> {
        self.colliders.get_mut(index)
    }

    pub fn update(&mut self, delta_time: f64) {
        self.collisions.clear();

        for a in 0..self.bodies.len() {
            for b in a + 1..self.bodies.len() {
                let body_a = &self.bodies[a];
                let body_b = &self.bodies[b];

                if body_a.has_finite_mass() || body_b.has_finite_mass() {
                    let collider_a = &self.colliders[a];
                    let collider_b = &self.colliders[b];

                    if let Some(manifold) = test_collision(
                        collider_a,
                        &body_a.transform_matrix,
                        collider_b,
                        &body_b.transform_matrix
                    ) {
                        self.collisions.push((manifold, a, b));
                    }
                }
            }
        }
        for (manifold, a, b) in self.collisions.iter() {
            let body_a = &self.bodies[*a];
            let body_b = &self.bodies[*b];

            let resolution_data = resolve_collision(manifold, body_a, body_b);

            let body_a = &mut self.bodies[*a];

            if body_a.has_finite_mass() {
                body_a.linear_velocity += body_a.inverse_mass * resolution_data.linear_impulse;
                body_a.angular_velocity += body_a.inverse_inertia_tensor_world * resolution_data.angular_impulse_a;
                body_a.position += body_a.inverse_mass * resolution_data.correction;
            }

            let body_b = &mut self.bodies[*b];

            if body_b.has_finite_mass() {
                body_b.linear_velocity -= body_b.inverse_mass * resolution_data.linear_impulse;
                body_b.angular_velocity -= body_b.inverse_inertia_tensor_world * resolution_data.angular_impulse_b;
                body_b.position -= body_b.inverse_mass * resolution_data.correction;
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