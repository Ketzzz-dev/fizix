use nalgebra::{Matrix3, Matrix4, Point3, Quaternion, Vector3, zero};
use crate::collisions::Collider;

pub struct RigidBody {
    pub position: Point3<f64>,
    pub orientation: Quaternion<f64>,

    pub linear_velocity: Vector3<f64>,
    pub rotational_velocity: Vector3<f64>,

    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,

    pub inverse_mass: f64,

    pub inverse_inertia_tensor_local: Matrix3<f64>,
    pub inverse_inertia_tensor_world: Matrix3<f64>,

    pub transform: Matrix4<f64>
}

pub struct DynamicRigidBodySettings {
    pub position: Point3<f64>,
    pub orientation: Quaternion<f64>,

    pub linear_velocity: Vector3<f64>,
    pub rotational_velocity: Vector3<f64>,

    pub mass: f64,
}
pub struct StaticRigidBodySettings {
    pub position: Point3<f64>,
    pub orientation: Quaternion<f64>
}

impl RigidBody {
    pub fn dynamic_body(settings: DynamicRigidBodySettings, collider: &impl Collider) -> Self {
        let mut rigid_body = Self {
            position: settings.position,
            orientation: settings.orientation,

            linear_velocity: settings.linear_velocity,
            rotational_velocity: settings.rotational_velocity,

            force: zero(),
            torque: zero(),

            inverse_mass: 1.0 / settings.mass,

            inverse_inertia_tensor_local: collider.calculate_inverse_inertia_tensor(settings.mass),
            inverse_inertia_tensor_world: Matrix3::identity(),

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
            rotational_velocity: zero(),

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
        let linear_acceleration = self.inverse_mass * self.force;
        let rotational_acceleration = self.inverse_inertia_tensor_world * self.torque;

        self.linear_velocity += linear_acceleration * delta_time;
        self.rotational_velocity += rotational_acceleration * delta_time;

        self.position += self.linear_velocity * delta_time;
        self.orientation += delta_time * 0.5 * Quaternion::new(0.0, self.rotational_velocity.x, self.rotational_velocity.y, self.rotational_velocity.z) * self.orientation;

        self.orientation.normalize_mut();

        self.force = zero();
        self.torque = zero();

        self.calculate_transform();
        self.calculate_inertia_tensor();
    }

    fn calculate_transform(&mut self) {
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
            self.transform.m11, self.transform.m12, self.transform.m13,
            self.transform.m21, self.transform.m22, self.transform.m23,
            self.transform.m31, self.transform.m32, self.transform.m33
        );

        self.inverse_inertia_tensor_world = rotation_matrix * self.inverse_inertia_tensor_local * rotation_matrix.transpose();
    }
}

impl Default for DynamicRigidBodySettings {
    fn default() -> Self {
        Self {
            position: Point3::origin(),
            orientation: Quaternion::identity(),

            linear_velocity: zero(),
            rotational_velocity: zero(),

            mass: 1.0
        }
    }
}
impl Default for StaticRigidBodySettings {
    fn default() -> Self {
        Self {
            position: Point3::origin(),
            orientation: Quaternion::identity()
        }
    }
}