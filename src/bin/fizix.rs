use std::time::Instant;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Translation3};
use kiss3d::text::Font;
use kiss3d::window::Window;
use nalgebra::{Point3, Vector3};
use fizix::constraints::{DistanceConstraint, FixedConstraint};
use fizix::particle_system::ParticleSystem;
use fizix::Scalar;

const ORANGE: (f32, f32, f32) = (244.0 / 255.0, 115.9 / 255.0, 51.0 / 255.0);
const GRAY: (f32, f32, f32) = (65.0 / 255.0, 67.0 / 255.0, 86.0 / 255.0);

fn main() {
    let mut window = Window::new("Fizix");
    let mut system = ParticleSystem::new(Vector3::new(0.0, -9.81, 0.0), 24, 1);

    window.set_light(Light::StickToCamera);
    window.set_background_color(24.0  / 255.0, 24.0 / 255.0, 37.0 / 255.0);

    let width = 50;
    let height = 50;
    let spacing = 0.25;

    for y in 0..height {
        for x in 0..width {
            let position = Point3::new(
                (x - width / 2) as Scalar * spacing,
                (height / 2) as Scalar * spacing,
                10.0 + (y + height / 2) as Scalar * spacing
            );
            let particle = system.add_particle(position, 1.0);

            if x > 0 {
                let previous_particle = y * width + x - 1;
                system.add_constraint(DistanceConstraint::new(previous_particle as usize, particle, spacing));
            }

            if y > 0 {
                let previous_particle = (y - 1) * width + x;
                system.add_constraint(DistanceConstraint::new(previous_particle as usize, particle, spacing));
            }

            if y == height - 1 && (x == 0 || x == width - 1) {
                system.add_constraint(FixedConstraint::new(particle, position));
            }
        }
    }

    system.save_state();

    let mut nodes = vec![];

    for _ in 0..system.state.x.len() {
        let mut node = window.add_sphere(0.1);
        node.set_color(ORANGE.0, ORANGE.1, ORANGE.2);

        nodes.push(node);
    }
    
    let mut last_time = Instant::now();

    let mut fps_samples = [0.0; 30];
    let mut sample_index = 0;

    while window.render() {
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::R, Action::Press, _) => {
                    println!("Resetting simulation");

                    system.reset();
                },

                _ => {}
            }
        }

        let elapsed = last_time.elapsed().as_secs_f32();

        last_time = Instant::now();

        fps_samples[sample_index] = elapsed;
        sample_index = (sample_index + 1) % fps_samples.len();

        system.update(elapsed);

        let ode_solve_time = system.get_ode_solve_time();
        let constraints_solve_time = system.get_constraints_solve_time();
        let velocity_update_time = system.get_velocity_update_time();
        let sub_step_iteration_time = system.get_sub_step_iteration_time();
        let fps = fps_samples.len() as f32 / fps_samples.iter().sum::<f32>();

        window.draw_text(
            &format!(
                "ODE Solve Time: {:.1?}\nConstraints Solve Time: {:.1?}\nVelocity Update Time: {:.1?}\nSub-step Iteration Time: {:.1?}\n\nParticle Count: {}\nFPS: {:.0}",
                ode_solve_time, constraints_solve_time, velocity_update_time, sub_step_iteration_time, system.state.x.len(), fps
            ),
            &Point2::new(10.0, 10.0),
            42.0,
            &Font::default(),
            &kiss3d::nalgebra::Point3::new(GRAY.0, GRAY.1, GRAY.2),
        );
        
        for (i, node) in nodes.iter_mut().enumerate() {
            let position = system.state.x[i];
            node.set_local_translation(Translation3::new(position.x, position.y, position.z));
        }
    }
}