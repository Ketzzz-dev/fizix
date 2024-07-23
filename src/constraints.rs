use nalgebra::{Point3, Quaternion};
use crate::rigid_body_system::State;
use crate::Scalar;

pub trait Constraint {
    fn project_rigid_bodies(&self, state: &mut State);
}

pub struct DistanceConstraint {
    rigid_body_a: usize,
    rigid_body_b: usize,

    local_point_a: Point3<Scalar>,
    local_point_b: Point3<Scalar>,

    distance: Scalar
}
impl DistanceConstraint {
    pub fn new(rigid_body_a: usize, rigid_body_b: usize, local_point_a: Point3<Scalar>, local_point_b: Point3<Scalar>, distance: Scalar) -> Self {
        Self {
            rigid_body_a,
            rigid_body_b,

            local_point_a,
            local_point_b,

            distance
        }
    }
}

impl Constraint for DistanceConstraint {
    fn project_rigid_bodies(&self, state: &mut State) {
        let world_point_a = state.transform[self.rigid_body_a] * self.local_point_a;
        let world_point_b = state.transform[self.rigid_body_b] * self.local_point_b;

        let delta = world_point_a - world_point_b;
        let distance = delta.norm();

        let error = distance - self.distance;
        let normal = delta / distance;

        let relative_point_a = world_point_a - state.position[self.rigid_body_a];
        let relative_point_b = world_point_b - state.position[self.rigid_body_b];

        let perpendicular_a = relative_point_a.cross(&normal);
        let perpendicular_b = relative_point_b.cross(&normal);

        let inverse_mass_a = state.inverse_mass[self.rigid_body_a]
            + perpendicular_a.dot(&(state.inverse_inertia_tensor_world[self.rigid_body_a] * perpendicular_a));
        let inverse_mass_b = state.inverse_mass[self.rigid_body_b]
            + perpendicular_b.dot(&(state.inverse_inertia_tensor_world[self.rigid_body_b] * perpendicular_b));

        let lambda = -error / (inverse_mass_a + inverse_mass_b);
        let positional_correction = normal * lambda;

        if state.has_finite_mass(self.rigid_body_a) {
            let rotational_correction = relative_point_a.cross(&positional_correction);
            let orientation = state.orientation[self.rigid_body_a];

            state.position[self.rigid_body_a] += state.inverse_mass[self.rigid_body_a] * positional_correction;
            state.orientation[self.rigid_body_a] += 0.5
                * Quaternion::from_imag(state.inverse_inertia_tensor_world[self.rigid_body_a] * rotational_correction)
                * orientation;

            state.calculate_derived_data(self.rigid_body_a);
        }
        if state.has_finite_mass(self.rigid_body_b) {
            let rotational_correction = relative_point_b.cross(&positional_correction);
            let orientation = state.orientation[self.rigid_body_b];

            state.position[self.rigid_body_b] -= state.inverse_mass[self.rigid_body_b] * positional_correction;
            state.orientation[self.rigid_body_b] -= 0.5
                * Quaternion::from_imag(state.inverse_inertia_tensor_world[self.rigid_body_b] * rotational_correction)
                * orientation;

            state.calculate_derived_data(self.rigid_body_b);
        }
    }
}

pub struct SphericalConstraint {
    rigid_body_a: usize,
    rigid_body_b: usize,

    local_point_a: Point3<Scalar>,
    local_point_b: Point3<Scalar>
}
impl SphericalConstraint {
    pub fn new(rigid_body_a: usize, rigid_body_b: usize, local_point_a: Point3<Scalar>, local_point_b: Point3<Scalar>) -> Self {
        Self {
            rigid_body_a,
            rigid_body_b,

            local_point_a,
            local_point_b
        }
    }
}

impl Constraint for SphericalConstraint {
    fn project_rigid_bodies(&self, state: &mut State) {
        let world_point_a = state.transform[self.rigid_body_a] * self.local_point_a;
        let world_point_b = state.transform[self.rigid_body_b] * self.local_point_b;

        let delta = world_point_a - world_point_b;

        let error = delta.norm();
        let normal = delta / error;

        let relative_point_a = world_point_a - state.position[self.rigid_body_a];
        let relative_point_b = world_point_b - state.position[self.rigid_body_b];

        let perpendicular_a = relative_point_a.cross(&normal);
        let perpendicular_b = relative_point_b.cross(&normal);

        let inverse_mass_a = state.inverse_mass[self.rigid_body_a]
            + perpendicular_a.dot(&(state.inverse_inertia_tensor_world[self.rigid_body_a] * perpendicular_a));
        let inverse_mass_b = state.inverse_mass[self.rigid_body_b]
            + perpendicular_b.dot(&(state.inverse_inertia_tensor_world[self.rigid_body_b] * perpendicular_b));

        let lambda = -error / (inverse_mass_a + inverse_mass_b);
        let positional_correction = normal * lambda;

        if state.has_finite_mass(self.rigid_body_a) {
            let rotational_correction = relative_point_a.cross(&positional_correction);
            let orientation = state.orientation[self.rigid_body_a];

            state.position[self.rigid_body_a] += state.inverse_mass[self.rigid_body_a] * positional_correction;
            state.orientation[self.rigid_body_a] += 0.5
                * Quaternion::from_imag(state.inverse_inertia_tensor_world[self.rigid_body_a] * rotational_correction)
                * orientation;

            state.calculate_derived_data(self.rigid_body_a);
        }
        if state.has_finite_mass(self.rigid_body_b) {
            let rotational_correction = relative_point_b.cross(&positional_correction);
            let orientation = state.orientation[self.rigid_body_b];

            state.position[self.rigid_body_b] -= state.inverse_mass[self.rigid_body_b] * positional_correction;
            state.orientation[self.rigid_body_b] -= 0.5
                * Quaternion::from_imag(state.inverse_inertia_tensor_world[self.rigid_body_b] * rotational_correction)
                * orientation;

            state.calculate_derived_data(self.rigid_body_b);
        }
    }
}