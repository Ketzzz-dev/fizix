use three_d::{WindowSettings, Window, Srgba, PhysicalMaterial, Mesh, Gm, FrameOutput, DirectionalLight, CpuMesh, CpuMaterial, ClearState, Camera, vec3, degrees, AmbientLight, Matrix4, Event, SquareMatrix};
use crate::collisions::{CollisionDetector, Box, Sphere};
use crate::dynamics::{RigidBody, World};
use crate::free_cam_control::FreeCamControl;
use crate::math::{Vector3, Quaternion};

pub mod dynamics;
pub mod math;
pub mod collisions;
mod free_cam_control;

const DELTA_TIME: f64 = 1.0 / 120.0;

fn main() {
    let window = Window::new(WindowSettings {
        title: String::from("test"),
        max_size: Some((1280, 720)),

        ..Default::default()
    }).unwrap();

    let context = window.gl();

    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(0.0, 0.0, 20.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(90.0),
        0.1,
        1000.0,
    );
    let mut controls = FreeCamControl::new(5.0, 2.5, 10.0);

    let mut cube = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::GREEN,

            ..Default::default()
        })
    );
    let mut sphere = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::BLUE,

            ..Default::default()
        })
    );
    let mut plane = Gm::new(
        Mesh::new(&context, &CpuMesh::square()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::WHITE,

            ..Default::default()
        })
    );

    plane.set_transformation(
        Matrix4::from_translation(vec3(0.0, -5.0, 0.0)) *
            Matrix4::from_angle_x(degrees(90.0)) *
            Matrix4::from_scale(25.0)
    );

    let mut sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let mut world = World::new();
    let mut body_a = RigidBody::new(Vector3 { x: -1.0, y: 0.0, z: 0.0 }, Quaternion::IDENTITY, 10.0);
    let mut body_b = RigidBody::new(Vector3 { x: 1.0, y: 0.0, z: 0.0 }, Quaternion::IDENTITY, 10.0);

    body_a.rotational_velocity = Vector3 { x: -1.0, y: -0.5, z: 0.25 } * 0.25;
    body_b.rotational_velocity = Vector3 { x: -0.25, y: 0.5, z: 1.0 } * 0.25;

    let body_index_a = world.add_body(body_a);
    let body_index_b = world.add_body(body_b);

    let mut accumulated_time = 0.0;

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        handle_events(&mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        while accumulated_time > DELTA_TIME {
            controls.update(&mut camera, DELTA_TIME);

            let body_a = world.get_body(body_index_a).unwrap();
            let body_b = world.get_body(body_index_b).unwrap();

            let primitive_a = Box { half_size: Vector3 { x: 2.0, y: 1.0, z: 3.0 } };
            let primitive_b = Sphere { radius: 1.0 };

            if let Some(_) = CollisionDetector::box_sphere(&primitive_a, body_a, &primitive_b, body_b) {
                cube.material.albedo = Srgba::RED;
                sphere.material.albedo = Srgba::RED;
            } else {
                cube.material.albedo = Srgba::GREEN;
                sphere.material.albedo = Srgba::BLUE;
            }

            world.update(DELTA_TIME);

            accumulated_time -= DELTA_TIME;
        }

        let body_a = world.get_body(body_index_a).unwrap();
        let body_b = world.get_body(body_index_b).unwrap();

        let transform_a = Matrix4::new(
            body_a.transform.m11 as f32, body_a.transform.m21 as f32, body_a.transform.m31 as f32, body_a.transform.m41 as f32,
            body_a.transform.m12 as f32, body_a.transform.m22 as f32, body_a.transform.m32 as f32, body_a.transform.m42 as f32,
            body_a.transform.m13 as f32, body_a.transform.m23 as f32, body_a.transform.m33 as f32, body_a.transform.m43 as f32,
            body_a.transform.m14 as f32, body_a.transform.m24 as f32, body_a.transform.m34 as f32, body_a.transform.m44 as f32
        );
        let transform_b = Matrix4::new(
            body_b.transform.m11 as f32, body_b.transform.m21 as f32, body_b.transform.m31 as f32, body_b.transform.m41 as f32,
            body_b.transform.m12 as f32, body_b.transform.m22 as f32, body_b.transform.m32 as f32, body_b.transform.m42 as f32,
            body_b.transform.m13 as f32, body_b.transform.m23 as f32, body_b.transform.m33 as f32, body_b.transform.m43 as f32,
            body_b.transform.m14 as f32, body_b.transform.m24 as f32, body_b.transform.m34 as f32, body_b.transform.m44 as f32
        );

        cube.set_transformation(transform_a * Matrix4::from_nonuniform_scale(2.0, 1.0, 3.0));
        sphere.set_transformation(transform_b);

        sun.generate_shadow_map(1024, cube.into_iter().chain(&sphere));

        frame_input.screen()
            .clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0))
            .render(&camera, cube.into_iter().chain(&sphere), &[&sun, &ambient]);

        FrameOutput::default()
    });
}

fn handle_events(events: &mut [Event]) {
    for event in events {
        match *event {
            _ => {}
        }
    }
}