use nalgebra::{Point3, Vector3};
use crate::particle_system::State;
use crate::Scalar;

// error and gradients
pub type ConstraintValues = (Scalar, Vec<Vector3<Scalar>>);

pub trait Constraint {
    fn calculate_values(&self, state: &State) -> ConstraintValues;
    fn get_particles(&self) -> Vec<usize>;
}

pub struct FixedConstraint {
    particle: usize,

    pub position: Point3<Scalar>
}
impl FixedConstraint {
    pub fn new(particle: usize, position: Point3<Scalar>) -> Self {
        Self { particle, position }
    }
}

impl Constraint for FixedConstraint {
    fn calculate_values(&self, state: &State) -> ConstraintValues {
        let difference = state.x[self.particle] - self.position;
        let distance_sq = difference.norm_squared();

        if distance_sq == 0.0 {
            return (0.0, vec![Vector3::zeros()]);
        }

        let distance = distance_sq.sqrt();

        (distance, vec![difference / distance])
    }

    fn get_particles(&self) -> Vec<usize> {
        vec![self.particle]
    }
}

pub struct DistanceConstraint {
    particle_a: usize,
    particle_b: usize,

    pub distance: Scalar
}
impl DistanceConstraint {
    pub fn new(particle_a: usize, particle_b: usize, distance: Scalar) -> Self {
        Self { particle_a, particle_b, distance }
    }
}

impl Constraint for DistanceConstraint {
    fn calculate_values(&self, state: &State) -> ConstraintValues {
        let position_a = state.x[self.particle_a];
        let position_b = state.x[self.particle_b];

        let difference = position_a - position_b;
        let distance_sq = difference.norm_squared();

        if distance_sq == 0.0 {
            return (-self.distance, vec![Vector3::x(), -Vector3::x()]);
        }

        let distance = distance_sq.sqrt();
        let gradient = difference / distance;

        (distance - self.distance, vec![gradient, -gradient])
    }
    fn get_particles(&self) -> Vec<usize> {
        vec![self.particle_a, self.particle_b]
    }
}