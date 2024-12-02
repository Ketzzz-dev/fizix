use nalgebra::{Isometry3, Matrix3, Point3, Quaternion, UnitQuaternion, Vector3};
use crate::constraints::Constraint;
use crate::Precision;

/// The data that represents the state of each Rigid Body in the system.
#[derive(Clone)]
pub struct State {
    pub position: Vec<Point3<Precision>>,
    pub orientation: Vec<Quaternion<Precision>>,

    pub last_position: Vec<Point3<Precision>>,
    pub last_orientation: Vec<Quaternion<Precision>>,

    pub linear_velocity: Vec<Vector3<Precision>>,
    pub angular_velocity: Vec<Vector3<Precision>>,

    pub force: Vec<Vector3<Precision>>,
    pub torque: Vec<Vector3<Precision>>,

    pub inverse_mass: Vec<Precision>,
    pub inverse_inertia_tensor: Vec<Matrix3<Precision>>,

    // derived data
    pub inverse_inertia_tensor_world: Vec<Matrix3<Precision>>,
    pub transform: Vec<Isometry3<Precision>>
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

            inverse_inertia_tensor_world: Vec::new(),
            transform: Vec::new()
        }
    }

    /// Checks if the Rigid Body at index `i` has finite mass (movable).
    pub fn has_finite_mass(&self, i: usize) -> bool {
        self.inverse_mass[i] > 0.0
    }

    /// Rotates the orientation of the Rigid Body at index `i`.
    pub fn rotate_orientation(&mut self, i: usize, rotation: Vector3<Precision>) {
        let orientation = self.orientation[i];

        self.orientation[i] += 0.5 * Quaternion::from_imag(rotation) * orientation;
        self.orientation[i].normalize_mut();
    }

    /// Calculates the derived data (Transform Matrix and Inverse Inertia Tensor) for the Rigid Body at index `i`.
    ///
    /// This function must be called whenever `position` or `orientation` has been manipulated.
    pub fn calculate_derived_data(&mut self, i: usize) {
        self.transform[i] = Isometry3::from_parts(self.position[i].into(), UnitQuaternion::from_quaternion(self.orientation[i]));

        let rotation = self.transform[i].rotation.to_rotation_matrix();

        self.inverse_inertia_tensor_world[i] = rotation * self.inverse_inertia_tensor[i] * rotation.transpose();
    }
}

/// The main system that handles the adding, removing, and updating of Rigid Bodies and Constraints.
pub struct RigidBodySystem {
    pub state: State,
    saved_state: Option<State>,

    pub constraints: Vec<Box<dyn Constraint>>,

    gravity: Vector3<Precision>,

    /// The amount to subdivide the physics steps while updating the system.
    sub_steps: usize,
    /// The amount of iterations to apply Constraints in a single step.
    constraint_iterations: usize
}
impl RigidBodySystem {
    pub fn new(gravity: Vector3<Precision>, sub_steps: usize, constraint_iterations: usize) -> Self {
        Self {
            state: State::new(),
            saved_state: None,

            constraints: Vec::new(),

            gravity,

            sub_steps,
            constraint_iterations,
        }
    }

    /// Saves the current state of the system.
    pub fn save_state(&mut self) {
        self.saved_state = Some(self.state.clone());
    }

    /// Resets the system's state to the last saved state.
    ///
    /// Does nothing if there is no saved state.
    pub fn reset(&mut self) {
        if let Some(initial_state) = &self.saved_state {
            self.state = initial_state.clone();
        }
    }

    /// Adds a new Rigid Body to the system defined by its `position`, `orientation`, `mass`, and `inertia_tensor`.
    ///
    /// If the `mass` supplied is infinite, the Rigid Body becomes static (immovable).
    ///
    /// Returns the index of the Rigid Body for direct access from the system.
    pub fn add_rigid_body(&mut self, position: Point3<Precision>, orientation: Quaternion<Precision>, mass: Precision, inertia_tensor: Matrix3<Precision>) -> usize {
        let transform = Isometry3::from_parts(position.into(), UnitQuaternion::from_quaternion(orientation));
        let rotation = transform.rotation.to_rotation_matrix();
        let inverse_inertia_tensor = if mass.is_infinite() {
            Matrix3::zeros()
        } else {
            inertia_tensor.try_inverse().unwrap_or(Matrix3::identity())
        };
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

    /// Adds the given Constraint to the system.
    ///
    /// The `constraint` supplied is moved to the heap and stores its pointer for reference.
    ///
    /// Returns the index of the Constraint for direct access from the system.
    pub fn add_constraint(&mut self, constraint: impl Constraint + 'static) -> usize {
        self.constraints.push(Box::new(constraint));

        self.constraints.len() - 1
    }

    /// Updates the system's state by the `elapsed_time`.
    ///
    /// Each physics step along with `elapsed_time` is subdivided into by the system's `sub_steps`.
    ///
    /// In each sub-step, the system first integrates each Rigid Body's position and velocity.
    /// Second, the Constraints are applied to each Rigid Body.
    /// Lastly, each Rigid Body's velocity is updated to match its projected (constrained) position.
    pub fn update(&mut self, elapsed_time: Precision) {
        let delta_time = elapsed_time / self.sub_steps as Precision;
        let inverse_delta_time = 1.0 / delta_time;

        for _ in 0..self.sub_steps {
            // pre-solve (integration)
            for i in 0..self.state.position.len() {
                if !self.state.has_finite_mass(i) { continue; }

                self.state.last_position[i] = self.state.position[i];
                self.state.last_orientation[i] = self.state.orientation[i];

                let linear_acceleration = self.gravity + self.state.inverse_mass[i] * self.state.force[i];
                let angular_acceleration = self.state.inverse_inertia_tensor_world[i] * self.state.torque[i];
                let orientation = self.state.orientation[i];

                self.state.force[i] = Vector3::zeros();
                self.state.linear_velocity[i] += linear_acceleration * delta_time;
                self.state.position[i] += self.state.linear_velocity[i] * delta_time;

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
                    constraint.project_bodies(&mut self.state);
                }
            }

            // post-solve (velocity update)
            for i in 0..self.state.position.len() {
                if !self.state.has_finite_mass(i) { continue; }

                let delta_orientation = self.state.orientation[i] * self.state.last_orientation[i].conjugate();

                self.state.linear_velocity[i] = (self.state.position[i] - self.state.last_position[i]) * inverse_delta_time;
                self.state.angular_velocity[i] = 2.0 * delta_orientation.imag() * inverse_delta_time;

                if delta_orientation.w < 0.0 {
                    self.state.angular_velocity[i].neg_mut();
                }
            }
        }
    }
}