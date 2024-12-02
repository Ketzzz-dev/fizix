use std::f64::consts::{SQRT_2};
use std::time::Instant;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Translation3, UnitQuaternion};
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::window::{Window};
use nalgebra::{Matrix3, Point3, Quaternion, Vector3};
use fizix::constraints::{AxisConstraint, LineConstraint, PointConstraint};
use fizix::Precision;
use fizix::rigid_body_system::RigidBodySystem;
use fizix::shapes::{cuboid_inertia_tensor};

const ORANGE: (f32, f32, f32) = (244.0 / 255.0, 115.9 / 255.0, 51.0 / 255.0); // primary color
const LIGHT_GRAY: (f32, f32, f32) = (108.0 / 255.0, 112.0 / 255.0, 134.0 / 255.0); // secondary color
const DARK_GRAY: (f32, f32, f32) = (49.0 / 255.0, 50.0 / 255.0, 68.0 / 255.0); // static color
const WHITE: (f32, f32, f32) = (204.0 / 255.0, 214.0 / 255.0, 244.0 / 255.0); // text color

fn main() {
    let mut window = Window::new_with_size("Fizix", 1280, 720);
    let mut system = RigidBodySystem::new(Vector3::new(0.0, -9.81 * 2.0, 0.0), 32, 4);

    window.set_light(Light::StickToCamera);
    window.set_background_color(24.0  / 255.0, 24.0 / 255.0, 37.0 / 255.0);

    system.save_state();

    let mut nodes: Vec<SceneNode> = vec![];

    let mut last_time = Instant::now();

    let mut fps_samples = [1.0 / 75.0; 75];
    let mut sample_index = 0;

    while window.render() {
        let elapsed = last_time.elapsed().as_secs_f64();

        last_time = Instant::now();
        fps_samples[sample_index] = elapsed;
        sample_index = (sample_index + 1) % fps_samples.len();

        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::R, Action::Press, _) => {
                    println!("Resetting simulation");

                    system.reset();
                }

                _ => {}
            }
        }

        system.update(elapsed);

        let fps = fps_samples.len() as f64 / fps_samples.iter().sum::<f64>();

        window.draw_text(
            &format!("Body Count: {}\nConstraint Count: {}\nFPS: {:.0}", system.state.position.len(), system.constraints.len(), fps),
            &Point2::new(10.0, 10.0),
            42.0,
            &Font::default(),
            &kiss3d::nalgebra::Point3::new(WHITE.0, WHITE.1, WHITE.2),
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