use three_d::{WindowSettings, Window, Srgba, PhysicalMaterial, OrbitControl, Mesh, Gm, FrameOutput, DirectionalLight, CpuMesh, CpuMaterial, ClearState, Camera, vec3, degrees, AmbientLight, Matrix4};
use crate::dynamics::{Gravity, RigidBody, World};
use crate::math::{Vector3};

pub mod dynamics;
pub mod math;

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
        degrees(75.0),
        0.1,
        1000.0,
    );
    let mut controls = OrbitControl::new(*camera.target(), 1.0, 100.0);

    let mut cube = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::RED,

            ..Default::default()
        })
    );

    let sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let mut world = World::new();

    let body = RigidBody::new(Vector3::ZERO, 10.0);
    let gravity = Gravity { gravity: Vector3 { x: 0.0, y: -9.81, z: 0.0 } };

    let body_index = world.add_body(body);
    let gravity_index = world.add_force_generator(gravity);

    world.set_pair(body_index, gravity_index);

    let mut accumulated_time = 0.0;

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        while accumulated_time > DELTA_TIME {
            world.update(DELTA_TIME);

            accumulated_time -= DELTA_TIME;
        }

        if let Some(body) = world.get_body(body_index) {
            cube.set_transformation(Matrix4::new(
                body.transform.m11 as f32, body.transform.m21 as f32, body.transform.m31 as f32, body.transform.m41 as f32,
                body.transform.m12 as f32, body.transform.m22 as f32, body.transform.m32 as f32, body.transform.m42 as f32,
                body.transform.m13 as f32, body.transform.m23 as f32, body.transform.m33 as f32, body.transform.m43 as f32,
                body.transform.m14 as f32, body.transform.m24 as f32, body.transform.m34 as f32, body.transform.m44 as f32
            ));
        }

        frame_input.screen()
            .clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0))
            .render(&camera, cube.into_iter(), &[&sun, &ambient]);

        FrameOutput::default()
    });
}
