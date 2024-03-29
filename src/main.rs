use three_d::{WindowSettings, Window, Srgba, PhysicalMaterial, Mesh, Gm, FrameOutput, DirectionalLight, CpuMesh, CpuMaterial, ClearState, Camera, vec3, degrees, AmbientLight, Matrix4};
use crate::collisions::{Collider, Cuboid, Plane, Sphere};
use crate::dynamics::{DynamicRigidBodySettings, RigidBody};

use crate::free_cam_control::FreeCamControl;
use crate::math::{Quaternion, Vector3};

pub mod math;
pub mod collisions;
pub mod dynamics;

mod free_cam_control;

const DELTA_TIME: f64 = 1.0 / 120.0;
const TIME_SCALE: f64 = 1.0 / 1.0;

fn main() {
    let window = Window::new(WindowSettings {
        title: String::from("test"),
        max_size: Some((1280, 720)),

        ..Default::default()
    }).unwrap();

    let context = window.gl();

    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(0.0, 10.0, 20.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(90.0),
        0.1,
        1000.0,
    );
    let mut controls = FreeCamControl::new(5.0, 2.5, 10.0);

    let mut gm_a = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::GREEN,

            ..Default::default()
        })
    );
    let mut gm_b = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::BLUE,

            ..Default::default()
        })
    );
    let mut gm_c = Gm::new(
        Mesh::new(&context, &CpuMesh::square()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::WHITE,

            ..Default::default()
        })
    );

    gm_c.set_transformation(
        Matrix4::from_translation(vec3(0.0, -5.0, 0.0)) *
            Matrix4::from_angle_x(degrees(90.0)) *
            Matrix4::from_scale(50.0)
    );

    let mut sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let collider_a = Sphere { radius: 1.0 };
    let collider_b = Cuboid { half_extents: Vector3 { x: 3.0, y: 1.0, z: 3.0 } };
    let collider_c = Plane { normal: Vector3::Y_AXIS, offset: -5.0 };

    let mut body_a = RigidBody::dynamic_body(DynamicRigidBodySettings {
        position: Vector3 { x: 1.0, y: 7.0, z: 1.0 },

        mass: 5.0,

        ..Default::default()
    }, &collider_a);
    let mut body_b = RigidBody::dynamic_body(DynamicRigidBodySettings {
        position: Vector3 { x: 0.0, y: 4.0, z: 0.0 },
        orientation: Quaternion { x: 1.0, y: 0.0, z: 1.0, w: 1.0 },

        mass: 1.0,

        ..Default::default()
    }, &collider_b);
    let mut body_c = RigidBody::static_body(Default::default());

    let gravity = Vector3 { x: 0.0, y:  -9.81, z: 0.0 };

    let mut accumulated_time = 0.0;

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        while accumulated_time > DELTA_TIME {
            controls.update(&mut camera, DELTA_TIME);

            // if let Some(collision_data) = collider_a.collides(&body_a.transform, &collider_b, &body_b.transform) {
            //     collision_data.solve(&mut body_a, &mut body_b);
            // }
            if let Some(collision_data) = collider_a.collides(&body_a.transform, &collider_c, &body_c.transform) {
                collision_data.solve(&mut body_a, &mut body_c);
            }
            if let Some(collision_data) = collider_b.collides(&body_b.transform, &collider_c, &body_c.transform) {
                collision_data.solve(&mut body_b, &mut body_c);
            }

            body_a.force += gravity * (1.0 / body_a.inverse_mass);
            body_b.force += gravity * (1.0 / body_b.inverse_mass);

            body_a.update(TIME_SCALE * DELTA_TIME);
            body_b.update(TIME_SCALE * DELTA_TIME);

            accumulated_time -= DELTA_TIME;
        }

        gm_a.set_transformation(Matrix4::new(
            body_a.transform.m11 as f32, body_a.transform.m21 as f32, body_a.transform.m31 as f32, body_a.transform.m41 as f32,
            body_a.transform.m12 as f32, body_a.transform.m22 as f32, body_a.transform.m32 as f32, body_a.transform.m42 as f32,
            body_a.transform.m13 as f32, body_a.transform.m23 as f32, body_a.transform.m33 as f32, body_a.transform.m43 as f32,
            body_a.transform.m14 as f32, body_a.transform.m24 as f32, body_a.transform.m34 as f32, body_a.transform.m44 as f32,
        ) * Matrix4::from_scale(collider_a.radius as f32));
        gm_b.set_transformation(Matrix4::new(
            body_b.transform.m11 as f32, body_b.transform.m21 as f32, body_b.transform.m31 as f32, body_b.transform.m41 as f32,
            body_b.transform.m12 as f32, body_b.transform.m22 as f32, body_b.transform.m32 as f32, body_b.transform.m42 as f32,
            body_b.transform.m13 as f32, body_b.transform.m23 as f32, body_b.transform.m33 as f32, body_b.transform.m43 as f32,
            body_b.transform.m14 as f32, body_b.transform.m24 as f32, body_b.transform.m34 as f32, body_b.transform.m44 as f32,
        ) * Matrix4::from_nonuniform_scale(
            collider_b.half_extents.x as f32,
            collider_b.half_extents.y as f32,
            collider_b.half_extents.z as f32
        ));

        sun.generate_shadow_map(1024, gm_a.into_iter().chain(&gm_b).chain(&gm_c));

        frame_input.screen()
            .clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0))
            .render(&camera, gm_a.into_iter().chain(&gm_b).chain(&gm_c), &[&sun, &ambient]);

        FrameOutput::default()
    });
}