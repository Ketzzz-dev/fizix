use crate::collisions::Collider;
use crate::math::{Matrix3x3, Matrix4x4, Quaternion, Vector3};

pub struct RigidBody {
    pub position: Vector3,
    pub orientation: Quaternion,

    pub linear_velocity: Vector3,
    pub rotational_velocity: Vector3,

    pub force: Vector3,
    pub torque: Vector3,

    pub inverse_mass: f64,

    pub inverse_inertia_tensor_local: Matrix3x3,
    pub inverse_inertia_tensor_world: Matrix3x3,

    pub transform: Matrix4x4
}

pub struct DynamicRigidBodySettings {
    pub position: Vector3,
    pub orientation: Quaternion,

    pub linear_velocity: Vector3,
    pub rotational_velocity: Vector3,

    pub mass: f64,
}
pub struct StaticRigidBodySettings {
    pub position: Vector3,
    pub orientation: Quaternion
}

impl RigidBody {
    pub fn dynamic_body(settings: DynamicRigidBodySettings, collider: &impl Collider) -> Self {
        let mut rigid_body = Self {
            position: settings.position,
            orientation: settings.orientation,

            linear_velocity: settings.linear_velocity,
            rotational_velocity: settings.rotational_velocity,

            force: Vector3::ZERO,
            torque: Vector3::ZERO,

            inverse_mass: 1.0 / settings.mass,

            inverse_inertia_tensor_local: collider.calculate_inverse_inertia_tensor(settings.mass),
            inverse_inertia_tensor_world: Matrix3x3::IDENTITY,

            transform: Matrix4x4::IDENTITY
        };

        rigid_body.calculate_transform();
        rigid_body.calculate_inertia_tensor();

        rigid_body
    }
    pub fn static_body(settings: StaticRigidBodySettings) -> Self {
        let mut rigid_body = Self {
            position: settings.position,
            orientation: settings.orientation,

            linear_velocity: Vector3::ZERO,
            rotational_velocity: Vector3::ZERO,

            force: Vector3::ZERO,
            torque: Vector3::ZERO,

            inverse_mass: 0.0,

            inverse_inertia_tensor_local: Matrix3x3::ZERO,
            inverse_inertia_tensor_world: Matrix3x3::ZERO,

            transform: Matrix4x4::IDENTITY
        };

        rigid_body.calculate_transform();
        rigid_body.calculate_inertia_tensor();

        rigid_body
    }

    pub fn update(&mut self, delta_time: f64) {
        let linear_acceleration = self.inverse_mass * self.force;
        let rotational_acceleration = self.inverse_inertia_tensor_world * self.torque;

        self.linear_velocity += linear_acceleration * delta_time;
        self.rotational_velocity += rotational_acceleration * delta_time;

        self.position += self.linear_velocity * delta_time;
        self.orientation *= Quaternion {
            w: 1.0,
            x: self.rotational_velocity.x * delta_time,
            y: self.rotational_velocity.y * delta_time,
            z: self.rotational_velocity.z * delta_time
        };

        self.orientation = self.orientation.normalized();

        self.force = Vector3::ZERO;
        self.torque = Vector3::ZERO;

        self.calculate_transform();
        self.calculate_inertia_tensor();
    }

    fn calculate_transform(&mut self) {
        let yy = self.orientation.y * self.orientation.y;
        let zz = self.orientation.z * self.orientation.z;
        let xy = self.orientation.x * self.orientation.y;
        let wz = self.orientation.w * self.orientation.z;
        let wy = self.orientation.w * self.orientation.y;
        let yz = self.orientation.y * self.orientation.z;
        let wx = self.orientation.w * self.orientation.x;
        let xx = self.orientation.x * self.orientation.x;
        let xz = self.orientation.x * self.orientation.z;

        self.transform.m11 = 1.0 - 2.0 * yy - 2.0 * zz;
        self.transform.m12 = 2.0 * xy - 2.0 * wz;
        self.transform.m13 = 2.0 * xz + 2.0 * wy;
        self.transform.m14 = self.position.x;

        self.transform.m21 = 2.0 * xy + 2.0 * wz;
        self.transform.m22 = 1.0 - 2.0 * xx - 2.0 * zz;
        self.transform.m23 = 2.0 * yz - 2.0 * wx;
        self.transform.m24 = self.position.y;

        self.transform.m31 = 2.0 * xz - 2.0 * wy;
        self.transform.m32 = 2.0 * yz + 2.0 * wx;
        self.transform.m33 = 1.0 - 2.0 * xx - 2.0 * yy;
        self.transform.m34 = self.position.z;
    }

    fn calculate_inertia_tensor(&mut self) {
        let rotation_matrix = self.transform.rotation();
        let inverse_inertia_tensor = rotation_matrix * self.inverse_inertia_tensor_local;

        self.inverse_inertia_tensor_world = inverse_inertia_tensor * rotation_matrix;
    }
}

impl Default for DynamicRigidBodySettings {
    fn default() -> Self {
        Self {
            position: Vector3::ZERO,
            orientation: Quaternion::IDENTITY,

            linear_velocity: Vector3::ZERO,
            rotational_velocity: Vector3::ZERO,

            mass: 1.0
        }
    }
}
impl Default for StaticRigidBodySettings {
    fn default() -> Self {
        Self {
            position: Vector3::ZERO,
            orientation: Quaternion::IDENTITY
        }
    }
}