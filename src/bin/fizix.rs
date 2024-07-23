use std::f64::consts::FRAC_1_SQRT_2;
use std::time::Instant;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Translation3, UnitQuaternion};
use kiss3d::text::Font;
use kiss3d::window::{CanvasSetup, NumSamples, Window};
use nalgebra::{Matrix3, Point3, Quaternion, Vector3};
use fizix::constraints::{SphericalConstraint};
use fizix::rigid_body_system::RigidBodySystem;

const ORANGE: (f32, f32, f32) = (244.0 / 255.0, 115.9 / 255.0, 51.0 / 255.0);
const GRAY: (f32, f32, f32) = (65.0 / 255.0, 67.0 / 255.0, 86.0 / 255.0);

fn main() {
    let mut window = Window::new_with_setup(
        "Fizix", 1280, 720,
        CanvasSetup {
            vsync: false,
            samples: NumSamples::One
        }
    );
    let mut system = RigidBodySystem::new(Vector3::new(0.0, -9.81 * 2.0, 0.0), 32, 1);

    window.set_light(Light::StickToCamera);
    window.set_background_color(24.0  / 255.0, 24.0 / 255.0, 37.0 / 255.0);

    let anchor = system.add_rigid_body(
        Point3::new(0.0, 5.0, 25.0),
        Quaternion::identity(),
        f64::INFINITY,
        Matrix3::zeros()
    );
    let link_a = system.add_rigid_body(
        Point3::new(2.0, 5.0, 25.0),
        Quaternion::new(FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2),
        1.0,
        Matrix3::from_diagonal(&Vector3::new(
            67.0 / 48.0,
            1.0 / 8.0,
            67.0 / 48.0
        ))
    );
    let link_b = system.add_rigid_body(
        Point3::new(6.0, 5.0, 25.0),
        Quaternion::new(FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2),
        1.0,
        Matrix3::from_diagonal(&Vector3::new(
            67.0 / 48.0,
            1.0 / 8.0,
            67.0 / 48.0
        ))
    );
    let weight = system.add_rigid_body(
        Point3::new(8.0, 5.0, 25.0),
        Quaternion::identity(),
        5.0,
        Matrix3::from_diagonal_element(5.0)
    );

    system.add_constraint(SphericalConstraint::new(anchor, link_a, Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 2.0, 0.0)));
    system.add_constraint(SphericalConstraint::new(link_a, link_b, Point3::new(0.0, -2.0, 0.0), Point3::new(0.0, 2.0, 0.0)));
    system.add_constraint(SphericalConstraint::new(link_b, weight, Point3::new(0.0, -2.0, 0.0), Point3::new(0.0, 0.0, 0.0)));

    system.state.linear_velocity[weight].z = 10.0;

    system.save_state();

    let mut nodes = vec![];

    let mut anchor_node = window.add_sphere(0.34);
    let mut link_node_a = window.add_capsule(0.25, 4.0);
    let mut link_node_b = window.add_capsule(0.25, 4.0);
    let mut weight_node = window.add_sphere(1.0);

    anchor_node.set_color(ORANGE.0, ORANGE.1, ORANGE.2);
    link_node_a.set_color(ORANGE.0, ORANGE.1, ORANGE.2);
    link_node_b.set_color(ORANGE.0, ORANGE.1, ORANGE.2);
    weight_node.set_color(ORANGE.0, ORANGE.1, ORANGE.2);

    nodes.push(anchor_node);
    nodes.push(link_node_a);
    nodes.push(link_node_b);
    nodes.push(weight_node);
    
    let mut last_time = Instant::now();

    let mut fps_samples = [0.0; 50];
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

        let elapsed = last_time.elapsed().as_secs_f64();

        last_time = Instant::now();
        fps_samples[sample_index] = elapsed;
        sample_index = (sample_index + 1) % fps_samples.len();

        system.update(elapsed);

        let fps = fps_samples.len() as f64 / fps_samples.iter().sum::<f64>();

        window.draw_text(
            &format!("Body Count: {}\nFPS: {:.0}", system.state.position.len(), fps),
            &Point2::new(10.0, 10.0),
            42.0,
            &Font::default(),
            &kiss3d::nalgebra::Point3::new(GRAY.0, GRAY.1, GRAY.2),
        );
        
        for (i, node) in nodes.iter_mut().enumerate() {
            let position = system.state.position[i];
            let orientation = system.state.orientation[i];
            let orientation = kiss3d::nalgebra::Quaternion::new(
                orientation.w as f32,
                orientation.i as f32,
                orientation.j as f32,
                orientation.k as f32
            );

            node.set_local_translation(Translation3::new(position.x as f32, position.y as f32, position.z as f32));
            node.set_local_rotation(UnitQuaternion::from_quaternion(orientation));
        }
    }
}