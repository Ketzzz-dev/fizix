use crate::math::{Matrix3x3, Matrix4x4, Quaternion, Vector3};

pub struct RigidBody {
    pub position: Vector3,
    pub orientation: Quaternion,

    pub linear_velocity: Vector3,
    pub rotational_velocity: Vector3,

    pub force: Vector3,
    pub torque: Vector3,

    pub linear_damping: f64,
    pub rotational_damping: f64,

    pub mass: f64,
    pub inverse_mass: f64,

    pub inverse_inertia_tensor_local: Matrix3x3,
    pub inverse_inertia_tensor_world: Matrix3x3,

    pub transform: Matrix4x4
}

pub struct World {
    gravity: Vector3,
    bodies: Vec<RigidBody>
}

impl RigidBody {
    pub fn new(position: Vector3, mass: f64) -> Self {
        let inverse_inertia = 3.0 / (2.0 * mass);

        // hardcoded into a 2m cube
        let inverse_inertia_tensor = Matrix3x3 {
            m11: inverse_inertia, m12: 0.0, m13: 0.0,
            m21: 0.0, m22: inverse_inertia, m23: 0.0,
            m31: 0.0, m32: 0.0, m33: inverse_inertia
        };

        Self {
            position,
            orientation: Quaternion::IDENTITY,

            linear_velocity: Vector3::ZERO,
            rotational_velocity: Vector3::ZERO,

            force: Vector3::ZERO,
            torque: Vector3::ZERO,

            linear_damping: 0.99,
            rotational_damping: 0.25,

            mass,
            inverse_mass: 1.0 / mass,

            inverse_inertia_tensor_world: inverse_inertia_tensor,
            inverse_inertia_tensor_local: inverse_inertia_tensor,

            transform: Matrix4x4::IDENTITY
        }
    }

    pub fn integrate(&mut self, delta_time: f64) {
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

        self.linear_velocity *= self.linear_damping.powf(delta_time);
        self.rotational_velocity *= self.rotational_damping.powf(delta_time);

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
        let rotation_matrix = Matrix3x3 {
            m11: self.transform.m11,
            m12: self.transform.m12,
            m13: self.transform.m13,

            m21: self.transform.m21,
            m22: self.transform.m22,
            m23: self.transform.m23,

            m31: self.transform.m31,
            m32: self.transform.m32,
            m33: self.transform.m33
        };
        let inverse_inertia_tensor = rotation_matrix * self.inverse_inertia_tensor_local;

        self.inverse_inertia_tensor_world = inverse_inertia_tensor * rotation_matrix;
    }
}

impl World {
    pub fn new() -> Self {
        Self {
            gravity: Vector3 { x: 0.0, y: -9.80665, z: 0.0 },
            bodies: vec![]
        }
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        self.bodies.push(body);

        self.bodies.len() - 1
    }
    pub fn get_body(&mut self, index: usize) -> Option<&mut RigidBody> {
        self.bodies.get_mut(index)
    }

    pub fn update(&mut self, dt: f64) {
        for body in &mut self.bodies {
            body.force += self.gravity * body.mass;

            body.integrate(dt);
        }
    }
}