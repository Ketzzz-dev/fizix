use nalgebra::{Point3, Vector3};
use crate::Precision;
use crate::rigid_body_system::State;

pub trait Constraint {
    fn project_bodies(&self, state: &mut State);
}

pub struct PointConstraint {
    body_a: usize, body_b: usize,

    local_point_a: Point3<Precision>,
    local_point_b: Point3<Precision>
}
impl PointConstraint {
    pub fn new(
        body_a: usize, body_b: usize,

        local_point_a: Point3<Precision>,
        local_point_b: Point3<Precision>
    ) -> Self {
        Self {
            body_a, body_b,

            local_point_a,
            local_point_b
        }
    }
}

impl Constraint for PointConstraint {
    fn project_bodies(&self, state: &mut State) {
        let world_point_a = state.transform[self.body_a] * self.local_point_a;
        let world_point_b = state.transform[self.body_b] * self.local_point_b;

        let difference = world_point_a - world_point_b;
        let distance = difference.norm();

        if distance == 0.0 { return; }

        let normal = difference / distance;

        let relative_point_a = world_point_a - state.position[self.body_a];
        let relative_point_b = world_point_b - state.position[self.body_b];

        let perpendicular_a = relative_point_a.cross(&normal);
        let perpendicular_b = relative_point_b.cross(&normal);

        let inverse_inertia_a = (perpendicular_a.transpose() * state.inverse_inertia_tensor_world[self.body_a] * perpendicular_a).x;
        let inverse_inertia_b = (perpendicular_b.transpose() * state.inverse_inertia_tensor_world[self.body_b] * perpendicular_b).x;

        let total_inverse_mass = state.inverse_mass[self.body_a] + state.inverse_mass[self.body_b] + inverse_inertia_a + inverse_inertia_b;

        let lambda = -distance / total_inverse_mass;
        let translational_correction = normal * lambda;

        if state.has_finite_mass(self.body_a) {
            let rotational_correction = relative_point_a.cross(&translational_correction);

            state.position[self.body_a] += state.inverse_mass[self.body_a] * translational_correction;

            state.rotate_by_vector(self.body_a, state.inverse_inertia_tensor_world[self.body_a] * rotational_correction);
            state.calculate_derived_data(self.body_a);
        }
        if state.has_finite_mass(self.body_b) {
            let rotational_correction = relative_point_b.cross(&translational_correction);

            state.position[self.body_b] -= state.inverse_mass[self.body_b] * translational_correction;

            state.rotate_by_vector(self.body_b, state.inverse_inertia_tensor_world[self.body_b] * -rotational_correction);
            state.calculate_derived_data(self.body_b);
        }
    }
}

pub struct LineConstraint {
    body_a: usize, body_b: usize,

    local_point_a: Point3<Precision>,
    local_point_b: Point3<Precision>,

    local_direction: Vector3<Precision> // relative to B
}
impl LineConstraint {
    pub fn new(
        body_a: usize, body_b: usize,

        local_point_a: Point3<Precision>,
        local_point_b: Point3<Precision>,

        local_direction: Vector3<Precision>
    ) -> Self {
        Self {
            body_a, body_b,

            local_point_a,
            local_point_b,

            local_direction
        }
    }
}

impl Constraint for LineConstraint {
    fn project_bodies(&self, state: &mut State) {
        let world_point_a = state.transform[self.body_a] * self.local_point_a;
        let world_point_b = state.transform[self.body_b] * self.local_point_b;
        let world_direction = state.transform[self.body_b] * self.local_direction;

        let projected_point = world_point_b + world_direction
            * world_direction.dot(&(world_point_a - world_point_b));

        let difference = world_point_a - projected_point;
        let distance = difference.norm();


        if distance == 0.0 { return; }

        let normal = difference / distance;

        let relative_point_a = world_point_a - state.position[self.body_a];
        let relative_point_b = projected_point - state.position[self.body_b];

        let perpendicular_a = relative_point_a.cross(&normal);
        let perpendicular_b = relative_point_b.cross(&normal);

        let inverse_inertia_a = (perpendicular_a.transpose() * state.inverse_inertia_tensor_world[self.body_a] * perpendicular_a).x;
        let inverse_inertia_b = (perpendicular_b.transpose() * state.inverse_inertia_tensor_world[self.body_b] * perpendicular_b).x;

        let total_inverse_mass = state.inverse_mass[self.body_a] + state.inverse_mass[self.body_b] + inverse_inertia_a + inverse_inertia_b;

        let lambda = -distance / total_inverse_mass;
        let translational_correction = normal * lambda;

        if state.has_finite_mass(self.body_a) {
            let rotational_correction = relative_point_a.cross(&translational_correction);

            state.position[self.body_a] += state.inverse_mass[self.body_a] * translational_correction;

            state.rotate_by_vector(self.body_a, state.inverse_inertia_tensor_world[self.body_a] * rotational_correction);
            state.calculate_derived_data(self.body_a);
        }
        if state.has_finite_mass(self.body_b) {
            let rotational_correction = relative_point_b.cross(&translational_correction);

            state.position[self.body_b] -= state.inverse_mass[self.body_b] * translational_correction;

            state.rotate_by_vector(self.body_b, state.inverse_inertia_tensor_world[self.body_b] * -rotational_correction);
            state.calculate_derived_data(self.body_b);
        }
    }
}

pub struct AxisConstraint {
    body_a: usize, body_b: usize,

    local_axis_a: Vector3<Precision>,
    local_axis_b: Vector3<Precision>
}
impl AxisConstraint {
    pub fn new(
        body_a: usize, body_b: usize,

        local_axis_a: Vector3<Precision>,
        local_axis_b: Vector3<Precision>
    ) -> Self {
        Self {
            body_a, body_b,

            local_axis_a,
            local_axis_b
        }
    }
}

impl Constraint for AxisConstraint {
    fn project_bodies(&self, state: &mut State) {
        let world_axis_a = state.transform[self.body_a] * self.local_axis_a;
        let world_axis_b = state.transform[self.body_b] * self.local_axis_b;

        let orthogonal = world_axis_a.cross(&world_axis_b);
        let magnitude = orthogonal.norm();

        if magnitude == 0.0 { return; }

        let normal = orthogonal / magnitude;
        let normal_transpose = normal.transpose();

        let inverse_inertia_a = (normal_transpose * state.inverse_inertia_tensor_world[self.body_a] * normal).x;
        let inverse_inertia_b = (normal_transpose * state.inverse_inertia_tensor_world[self.body_b] * normal).x;

        let total_inverse_mass = inverse_inertia_a + inverse_inertia_b;

        let lambda = -magnitude / total_inverse_mass;
        let rotational_correction = normal * lambda;

        if state.has_finite_mass(self.body_a) {
            state.rotate_by_vector(self.body_a, state.inverse_inertia_tensor_world[self.body_a] * rotational_correction);
            state.calculate_derived_data(self.body_a);
        }
        if state.has_finite_mass(self.body_b) {
            state.rotate_by_vector(self.body_b, state.inverse_inertia_tensor_world[self.body_b] * -rotational_correction);
            state.calculate_derived_data(self.body_b);
        }
    }
}