use std::f64::consts::{SQRT_2};
use std::time::Instant;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Translation3, UnitQuaternion};
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
    let mut system = RigidBodySystem::new(Vector3::new(0.0, -9.81 * 2.0, 0.0), 16, 8);

    window.set_light(Light::StickToCamera);
    window.set_background_color(24.0  / 255.0, 24.0 / 255.0, 37.0 / 255.0);

    let anchor_a = system.add_rigid_body(
        Point3::new(25.0, -5.0, 50.0),
        Quaternion::identity(),
        Precision::INFINITY,
        Matrix3::zeros()
    );
    let anchor_b = system.add_rigid_body(
        Point3::new(-25.0, -5.0, 50.0),
        Quaternion::identity(),
        Precision::INFINITY,
        Matrix3::zeros()
    );

    let road_a = system.add_rigid_body(
        Point3::new(20.0, -5.0, 50.0),
        Quaternion::identity(),
        5.0,
        cuboid_inertia_tensor(10.0, 1.0, 10.0, 5.0)
    );
    let road_b = system.add_rigid_body(
        Point3::new(10.0, -5.0, 50.0),
        Quaternion::identity(),
        5.0,
        cuboid_inertia_tensor(10.0, 1.0, 10.0, 5.0)
    );
    let road_c = system.add_rigid_body(
        Point3::new(0.0, -5.0, 50.0),
        Quaternion::identity(),
        5.0,
        cuboid_inertia_tensor(10.0, 1.0, 10.0, 5.0)
    );
    let road_d = system.add_rigid_body(
        Point3::new(-10.0, -5.0, 50.0),
        Quaternion::identity(),
        5.0,
        cuboid_inertia_tensor(10.0, 1.0, 10.0, 5.0)
    );
    let road_e = system.add_rigid_body(
        Point3::new(-20.0, -5.0, 50.0),
        Quaternion::identity(),
        5.0,
        cuboid_inertia_tensor(10.0, 1.0, 10.0, 5.0)
    );

    let truss_a = system.add_rigid_body(
        Point3::new(20.0, 0.0, 50.0),
        Quaternion::new(0.9238795, 0.0, 0.0, -0.3826834),
        1.0,
        cuboid_inertia_tensor(10.0 * SQRT_2, 1.0, 10.0, 1.0)
    );
    let truss_b = system.add_rigid_body(
        Point3::new(15.0, 0.0, 50.0),
        Quaternion::identity(),
        1.0,
        cuboid_inertia_tensor(1.0, 10.0, 10.0, 1.0)
    );
    let truss_c = system.add_rigid_body(
        Point3::new(10.0, 0.0, 50.0),
        Quaternion::new(0.9238795, 0.0, 0.0, 0.3826834),
        1.0,
        cuboid_inertia_tensor(10.0 * SQRT_2, 1.0, 10.0, 1.0)
    );
    let truss_d = system.add_rigid_body(
        Point3::new(10.0, 5.0, 50.0),
        Quaternion::identity(),
        1.0,
        cuboid_inertia_tensor(10.0, 1.0, 10.0, 1.0)
    );
    let truss_e = system.add_rigid_body(
        Point3::new(5.0, 0.0, 50.0),
        Quaternion::identity(),
        1.0,
        cuboid_inertia_tensor(1.0, 10.0, 10.0, 1.0)
    );
    let truss_f = system.add_rigid_body(
        Point3::new(0.0, 0.0, 50.0),
        Quaternion::new(0.9238795, 0.0, 0.0, 0.3826834),
        1.0,
        cuboid_inertia_tensor(10.0 * SQRT_2, 1.0, 10.0, 1.0)
    );
    let truss_g = system.add_rigid_body(
        Point3::new(0.0, 5.0, 50.0),
        Quaternion::identity(),
        1.0,
        cuboid_inertia_tensor(10.0, 1.0, 10.0, 1.0)
    );
    let truss_h = system.add_rigid_body(
        Point3::new(-5.0, 0.0, 50.0),
        Quaternion::identity(),
        1.0,
        cuboid_inertia_tensor(1.0, 10.0, 10.0, 1.0)
    );
    let truss_i = system.add_rigid_body(
        Point3::new(-10.0, 0.0, 50.0),
        Quaternion::new(0.9238795, 0.0, 0.0, -0.3826834),
        1.0,
        cuboid_inertia_tensor(10.0 * SQRT_2, 1.0, 10.0, 1.0)
    );
    let truss_j = system.add_rigid_body(
        Point3::new(-10.0, 5.0, 50.0),
        Quaternion::identity(),
        1.0,
        cuboid_inertia_tensor(10.0, 1.0, 10.0, 1.0)
    );
    let truss_k = system.add_rigid_body(
        Point3::new(-15.0, 0.0, 50.0),
        Quaternion::identity(),
        1.0,
        cuboid_inertia_tensor(1.0, 10.0, 10.0, 1.0)
    );
    let truss_l = system.add_rigid_body(
        Point3::new(-20.0, 0.0, 50.0),
        Quaternion::new(0.9238795, 0.0, 0.0, 0.3826834),
        1.0,
        cuboid_inertia_tensor(10.0 * SQRT_2, 1.0, 10.0, 1.0)
    );

    system.add_constraint(PointConstraint::new(
        anchor_a, road_a,
        Point3::origin(),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        anchor_a, road_a,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        road_a, road_b,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        road_a, road_b,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        road_b, road_c,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        road_b, road_c,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        road_c, road_d,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        road_c, road_d,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        road_d, road_e,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        road_d, road_e,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(LineConstraint::new(
        road_e, anchor_b,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::origin(),
        Vector3::x()
    ));
    system.add_constraint(AxisConstraint::new(
        road_e, anchor_b,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_a, road_a,
        Point3::new(5.0 * SQRT_2, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_a, road_a,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_a, truss_b,
        Point3::new(-5.0 * SQRT_2, 0.0, 0.0),
        Point3::new(0.0, 5.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_a, truss_b,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_b, road_b,
        Point3::new(0.0, -5.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_b, road_b,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_b, truss_c,
        Point3::new(0.0, 5.0, 0.0),
        Point3::new(5.0 * SQRT_2, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_b, truss_c,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_c, road_c,
        Point3::new(-5.0 * SQRT_2, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_c, road_c,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_c, truss_d,
        Point3::new(5.0 * SQRT_2, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_c, truss_d,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_d, truss_e,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::new(0.0, 5.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_d, truss_e,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_e, road_c,
        Point3::new(0.0, -5.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_e, road_c,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_e, truss_f,
        Point3::new(0.0, 5.0, 0.0),
        Point3::new(5.0 * SQRT_2, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_e, truss_f,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_f, road_d,
        Point3::new(-5.0 * SQRT_2, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_f, road_d,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_f, truss_g,
        Point3::new(5.0 * SQRT_2, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_f, truss_g,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_g, truss_h,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::new(0.0, 5.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_g, truss_h,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_h, road_d,
        Point3::new(0.0, -5.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_h, road_d,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_h, truss_i,
        Point3::new(0.0, -5.0, 0.0),
        Point3::new(5.0 * SQRT_2, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_h, truss_i,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_g, truss_j,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_g, truss_j,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_i, truss_j,
        Point3::new(-5.0 * SQRT_2, 0.0, 0.0),
        Point3::new(-5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_i, truss_j,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_j, truss_k,
        Point3::new(-5.0, 0.0, 0.0),
        Point3::new(0.0, 5.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_j, truss_k,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_k, road_e,
        Point3::new(0.0, -5.0, 0.0),
        Point3::new(5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_k, road_e,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_k, truss_l,
        Point3::new(0.0, 5.0, 0.0),
        Point3::new(5.0 * SQRT_2, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_k, truss_l,
        Vector3::z(),
        Vector3::z()
    ));

    system.add_constraint(PointConstraint::new(
        truss_l, road_e,
        Point3::new(-5.0 * SQRT_2, 0.0, 0.0),
        Point3::new(-5.0, 0.0, 0.0)
    ));
    system.add_constraint(AxisConstraint::new(
        truss_l, road_e,
        Vector3::z(),
        Vector3::z()
    ));

    system.save_state();

    let mut nodes = vec![];

    let mut anchor_a_node = window.add_cube(1.5, 1.5, 12.5);
    let mut anchor_b_node = window.add_cube(1.5, 1.5, 12.5);

    let mut road_a_node = window.add_cube(10.0, 1.0, 10.0);
    let mut road_b_node = window.add_cube(10.0, 1.0, 10.0);
    let mut road_c_node = window.add_cube(10.0, 1.0, 10.0);
    let mut road_d_node = window.add_cube(10.0, 1.0, 10.0);
    let mut road_e_node = window.add_cube(10.0, 1.0, 10.0);

    let mut truss_a_node = window.add_cube(10.0 * SQRT_2 as f32, 1.0, 10.0);
    let mut truss_b_node = window.add_cube(1.0, 10.0, 10.0);
    let mut truss_c_node = window.add_cube(10.0 * SQRT_2 as f32, 1.0, 10.0);
    let mut truss_d_node = window.add_cube(10.0, 1.0, 10.0);
    let mut truss_e_node = window.add_cube(1.0, 10.0, 10.0);
    let mut truss_f_node = window.add_cube(10.0 * SQRT_2 as f32, 1.0, 10.0);
    let mut truss_g_node = window.add_cube(10.0, 1.0, 10.0);
    let mut truss_h_node = window.add_cube(1.0, 10.0, 10.0);
    let mut truss_i_node = window.add_cube(10.0 * SQRT_2 as f32, 1.0, 10.0);
    let mut truss_j_node = window.add_cube(10.0, 1.0, 10.0);
    let mut truss_k_node = window.add_cube(1.0, 10.0, 10.0);
    let mut truss_l_node = window.add_cube(10.0 * SQRT_2 as f32, 1.0, 10.0);

    anchor_a_node.set_color(DARK_GRAY.0, DARK_GRAY.1, DARK_GRAY.2);
    anchor_b_node.set_color(DARK_GRAY.0, DARK_GRAY.1, DARK_GRAY.2);

    road_a_node.set_color(ORANGE.0, ORANGE.1, ORANGE.2);
    road_b_node.set_color(ORANGE.0, ORANGE.1, ORANGE.2);
    road_c_node.set_color(ORANGE.0, ORANGE.1, ORANGE.2);
    road_d_node.set_color(ORANGE.0, ORANGE.1, ORANGE.2);
    road_e_node.set_color(ORANGE.0, ORANGE.1, ORANGE.2);

    truss_a_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_b_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_c_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_d_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_e_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_f_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_g_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_h_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_i_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_j_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_k_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);
    truss_l_node.set_color(LIGHT_GRAY.0, LIGHT_GRAY.1, LIGHT_GRAY.2);

    nodes.push(anchor_a_node);
    nodes.push(anchor_b_node);

    nodes.push(road_a_node);
    nodes.push(road_b_node);
    nodes.push(road_c_node);
    nodes.push(road_d_node);
    nodes.push(road_e_node);

    nodes.push(truss_a_node);
    nodes.push(truss_b_node);
    nodes.push(truss_c_node);
    nodes.push(truss_d_node);
    nodes.push(truss_e_node);
    nodes.push(truss_f_node);
    nodes.push(truss_g_node);
    nodes.push(truss_h_node);
    nodes.push(truss_i_node);
    nodes.push(truss_j_node);
    nodes.push(truss_k_node);
    nodes.push(truss_l_node);

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