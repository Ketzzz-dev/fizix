use std::time::{Duration, Instant};
use nalgebra::{Point3, Vector3};
use crate::constraints::Constraint;
use crate::Scalar;

#[derive(Clone)]
pub struct State {
    pub x: Vec<Point3<Scalar>>, // Position
    pub p: Vec<Point3<Scalar>>, // Previous position
    pub v: Vec<Vector3<Scalar>>, // Velocity

    pub m: Vec<Scalar>, // Mass
    pub w: Vec<Scalar>, // Inverse mass
}
impl State {
    pub fn new() -> Self {
        Self {
            x: Vec::new(),
            p: Vec::new(),
            v: Vec::new(),

            m: Vec::new(),
            w: Vec::new(),
        }
    }
}

pub struct ParticleSystem {
    pub state: State,

    saved_state: Option<State>,

    pub constraints: Vec<Box<dyn Constraint>>,

    gravity: Vector3<Scalar>,

    sub_steps: usize,
    constraint_iterations: usize,

    sub_step_iteration_time_samples: [Duration; 60],
    ode_solve_time_samples: [Duration; 60],
    constraints_solve_time_samples: [Duration; 60],
    velocity_update_time_samples: [Duration; 60],

    sub_step_sample_index: usize,
    step_sample_index: usize
}
impl ParticleSystem {
    pub fn new(gravity: Vector3<Scalar>, sub_steps: usize, constraint_iterations: usize) -> Self {
        Self {
            state: State::new(),
            saved_state: None,

            constraints: Vec::new(),

            gravity,

            sub_steps,
            constraint_iterations,

            sub_step_iteration_time_samples: [Duration::default(); 60],
            ode_solve_time_samples: [Duration::default(); 60],
            constraints_solve_time_samples: [Duration::default(); 60],
            velocity_update_time_samples: [Duration::default(); 60],

            sub_step_sample_index: 0,
            step_sample_index: 0
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

    pub fn add_particle(&mut self, position: Point3<Scalar>, mass: Scalar) -> usize {
        self.state.x.push(position);
        self.state.p.push(position);
        self.state.v.push(Vector3::zeros());

        self.state.m.push(mass);
        self.state.w.push(1.0 / mass);

        self.state.x.len() - 1
    }

    pub fn add_constraint(&mut self, constraint: impl Constraint + 'static) {
        self.constraints.push(Box::new(constraint));
    }

    pub fn update(&mut self, elapsed_time: Scalar) {
        let delta_time = elapsed_time / self.sub_steps as Scalar;
        let inverse_delta_time = 1.0 / delta_time;

        let sub_step_start = Instant::now();

        for _ in 0..self.sub_steps {
            let ode_start = Instant::now();

            // pre-solve (integration)
            for i in 0..self.state.x.len() {
                self.state.p[i] = self.state.x[i];
                self.state.x[i] += self.state.v[i] * delta_time + 0.5 * self.gravity * delta_time * delta_time;
            }

            self.ode_solve_time_samples[self.step_sample_index] = ode_start.elapsed();

            let constraints_start = Instant::now();

            // solve (constraints)
            for _ in 0..self.constraint_iterations {
                for constraint in self.constraints.iter() {
                    let (error, gradients) = constraint.calculate_values(&self.state);
                    let particles = constraint.get_particles();

                    assert_eq!(gradients.len(), particles.len());

                    let inverse_mass_sum = particles.iter().map(|&i| self.state.w[i]).sum::<Scalar>();
                    let lambda = -error / inverse_mass_sum;

                    for (&particle, gradient) in particles.iter().zip(gradients) {
                        self.state.x[particle] += self.state.w[particle] * gradient * lambda;
                    }
                }
            }

            self.constraints_solve_time_samples[self.step_sample_index] = constraints_start.elapsed();

            let velocity_start = Instant::now();

            // post-solve (velocity update)
            for i in 0..self.state.x.len() {
                self.state.v[i] = (self.state.x[i] - self.state.p[i]) * inverse_delta_time;
            }

            self.velocity_update_time_samples[self.step_sample_index] = velocity_start.elapsed();
            self.step_sample_index = (self.step_sample_index + 1) % self.ode_solve_time_samples.len();
        }

        self.sub_step_iteration_time_samples[self.sub_step_sample_index] = sub_step_start.elapsed();
        self.sub_step_sample_index = (self.sub_step_sample_index + 1) % self.sub_step_iteration_time_samples.len();
    }

    pub fn get_sub_step_iteration_time(&self) -> Duration {
        self.sub_step_iteration_time_samples.iter().sum::<Duration>() / self.sub_step_iteration_time_samples.len() as u32
    }
    pub fn get_ode_solve_time(&self) -> Duration {
        self.ode_solve_time_samples.iter().sum::<Duration>() / self.ode_solve_time_samples.len() as u32
    }
    pub fn get_constraints_solve_time(&self) -> Duration {
        self.constraints_solve_time_samples.iter().sum::<Duration>() / self.constraints_solve_time_samples.len() as u32
    }
    pub fn get_velocity_update_time(&self) -> Duration {
        self.velocity_update_time_samples.iter().sum::<Duration>() / self.velocity_update_time_samples.len() as u32
    }
}