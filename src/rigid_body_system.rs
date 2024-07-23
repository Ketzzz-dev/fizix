use nalgebra::{Isometry3, Matrix3, Point3, Quaternion, UnitQuaternion, Vector3};
use crate::constraints::Constraint;
use crate::Scalar;

#[derive(Clone)]
pub struct State {
    pub position: Vec<Point3<Scalar>>,
    pub orientation: Vec<Quaternion<Scalar>>,

    pub last_position: Vec<Point3<Scalar>>,
    pub last_orientation: Vec<Quaternion<Scalar>>,

    pub linear_velocity: Vec<Vector3<Scalar>>,
    pub angular_velocity: Vec<Vector3<Scalar>>,

    pub force: Vec<Vector3<Scalar>>,
    pub torque: Vec<Vector3<Scalar>>,

    pub inverse_mass: Vec<Scalar>,

    pub inverse_inertia_tensor: Vec<Matrix3<Scalar>>,
    pub inverse_inertia_tensor_world: Vec<Matrix3<Scalar>>,

    pub transform: Vec<Isometry3<Scalar>>
}
impl State {
    pub fn new() -> Self {
        Self {
            position: Vec::new(),
            orientation: Vec::new(),

            last_position: Vec::new(),
            last_orientation: Vec::new(),

            linear_velocity: Vec::new(),
            angular_velocity: Vec::new(),

            force: Vec::new(),
            torque: Vec::new(),

            inverse_mass: Vec::new(),
            inverse_inertia_tensor: Vec::new(),

            // derived data
            inverse_inertia_tensor_world: Vec::new(),
            transform: Vec::new()
        }
    }

    pub fn has_finite_mass(&self, i: usize) -> bool {
        self.inverse_mass[i] > 0.0
    }
    pub fn calculate_derived_data(&mut self, i: usize) { // always call this after updating position or orientation
        self.transform[i] = Isometry3::from_parts(self.position[i].into(), UnitQuaternion::from_quaternion(self.orientation[i]));

        let rotation = self.transform[i].rotation.to_rotation_matrix();

        self.inverse_inertia_tensor_world[i] = rotation * self.inverse_inertia_tensor[i] * rotation.transpose();
    }
}

pub struct RigidBodySystem {
    pub state: State,

    saved_state: Option<State>,

    pub constraints: Vec<Box<dyn Constraint>>,

    gravity: Vector3<Scalar>,

    sub_steps: usize,
    constraint_iterations: usize
}
impl RigidBodySystem {
    pub fn new(gravity: Vector3<Scalar>, sub_steps: usize, constraint_iterations: usize) -> Self {
        Self {
            state: State::new(),
            saved_state: None,

            constraints: Vec::new(),

            gravity,

            sub_steps,
            constraint_iterations,
        }
    }

    pub fn save_state(&mut self) {
        self.saved_state = Some(self.state.clone());
    }
    pub fn reset(&mut self) {
        if let Some(initial_state) = &self.saved_state {
            self.state = initial_state.clone();
        }
    }

    pub fn add_rigid_body(&mut self, position: Point3<Scalar>, orientation: Quaternion<Scalar>, mass: Scalar, inertia_tensor: Matrix3<Scalar>) -> usize {
        let transform = Isometry3::from_parts(position.into(), UnitQuaternion::from_quaternion(orientation));
        let rotation = transform.rotation.to_rotation_matrix();
        let inverse_inertia_tensor = inertia_tensor.try_inverse().unwrap_or(Matrix3::zeros());
        let inverse_inertia_tensor_world = rotation * inverse_inertia_tensor * rotation.transpose();

        self.state.position.push(position);
        self.state.orientation.push(orientation);

        self.state.last_position.push(position);
        self.state.last_orientation.push(orientation);

        self.state.linear_velocity.push(Vector3::zeros());
        self.state.angular_velocity.push(Vector3::zeros());

        self.state.force.push(Vector3::zeros());
        self.state.torque.push(Vector3::zeros());

        self.state.inverse_mass.push(1.0 / mass);
        self.state.inverse_inertia_tensor.push(inverse_inertia_tensor);

        self.state.inverse_inertia_tensor_world.push(inverse_inertia_tensor_world);
        self.state.transform.push(transform);

        self.state.position.len() - 1
    }

    pub fn add_constraint(&mut self, constraint: impl Constraint + 'static) -> usize {
        self.constraints.push(Box::new(constraint));

        self.constraints.len() - 1
    }

    pub fn update(&mut self, elapsed_time: Scalar) {
        let delta_time = elapsed_time / self.sub_steps as Scalar;
        let inverse_delta_time = 1.0 / delta_time;

        for _ in 0..self.sub_steps {
            // pre-solve (integration)
            for i in 0..self.state.position.len() {
                if !self.state.has_finite_mass(i) { continue; }

                self.state.last_position[i] = self.state.position[i];
                self.state.last_orientation[i] = self.state.orientation[i];

                let linear_acceleration = self.gravity + self.state.inverse_mass[i] * self.state.force[i];
                let angular_acceleration = self.state.inverse_inertia_tensor_world[i] * self.state.torque[i];

                self.state.force[i] = Vector3::zeros();
                self.state.linear_velocity[i] += linear_acceleration * delta_time;
                self.state.position[i] += self.state.linear_velocity[i] * delta_time;

                let orientation = self.state.orientation[i];

                self.state.torque[i] = Vector3::zeros();
                self.state.angular_velocity[i] += angular_acceleration * delta_time;
                self.state.orientation[i] += 0.5 * Quaternion::from_imag(self.state.angular_velocity[i])
                    * orientation * delta_time;

                self.state.orientation[i].normalize_mut();
                self.state.calculate_derived_data(i);
            }

            // solve (constraints)
            for _ in 0..self.constraint_iterations {
                for constraint in self.constraints.iter() {
                    constraint.project_rigid_bodies(&mut self.state);
                }
            }

            // post-solve (velocity update)
            for i in 0..self.state.position.len() {
                if !self.state.has_finite_mass(i) { continue; }

                self.state.linear_velocity[i] = (self.state.position[i] - self.state.last_position[i]) * inverse_delta_time;

                let delta_orientation = self.state.orientation[i] * self.state.last_orientation[i].conjugate();

                self.state.angular_velocity[i] = 2.0 * delta_orientation.imag() * inverse_delta_time;

                if delta_orientation.w < 0.0 {
                    self.state.angular_velocity[i].neg_mut();
                }
            }
        }
    }
}