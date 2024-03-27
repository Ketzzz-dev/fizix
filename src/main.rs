use three_d::{WindowSettings, Window, Srgba, PhysicalMaterial, Mesh, Gm, FrameOutput, DirectionalLight, CpuMesh, CpuMaterial, ClearState, Camera, vec3, degrees, AmbientLight, Matrix4};
use crate::collisions::{Collider, Plane, Sphere};
use crate::dynamics::{DynamicRigidBodySettings, RigidBody};

use crate::free_cam_control::FreeCamControl;
use crate::math::{Vector3};

pub mod math;
pub mod collisions;

mod free_cam_control;
mod dynamics;

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
        vec3(0.0, 10.0, 20.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(90.0),
        0.1,
        1000.0,
    );
    let mut controls = FreeCamControl::new(5.0, 2.5, 10.0);

    let mut sphere_gm_a = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::GREEN,

            ..Default::default()
        })
    );
    let mut sphere_gm_b = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::BLUE,

            ..Default::default()
        })
    );
    let mut plane_gm = Gm::new(
        Mesh::new(&context, &CpuMesh::square()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::WHITE,

            ..Default::default()
        })
    );

    plane_gm.set_transformation(
        Matrix4::from_translation(vec3(0.0, -5.0, 0.0)) *
            Matrix4::from_angle_x(degrees(90.0)) *
            Matrix4::from_scale(50.0)
    );

    let mut sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let sphere_collider_a = Sphere { radius: 1.0 };
    let sphere_collider_b = Sphere { radius: 1.5 };
    let plane_collider = Plane { normal: Vector3::Y_AXIS, offset: -5.0 };

    let mut sphere_rigid_body_a = RigidBody::dynamic_body(DynamicRigidBodySettings {
        position: Vector3 { x: 0.0, y: 5.0, z: 0.0 },

        mass: 1.0,

        ..Default::default()
    }, &sphere_collider_a);
    let mut sphere_rigid_body_b = RigidBody::dynamic_body(DynamicRigidBodySettings {
        position: Vector3 { x: 0.0, y: 1.0, z: 0.0 },

        mass: 10.0,

        ..Default::default()
    }, &sphere_collider_b);
    let mut plane_rigid_body = RigidBody::static_body(Default::default());

    let gravity = Vector3 { x: 0.0, y:  -9.81, z: 0.0 };

    let mut accumulated_time = 0.0;

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        while accumulated_time > DELTA_TIME {
            controls.update(&mut camera, DELTA_TIME);

            if let Some(collision_data) = sphere_collider_a.collides(&sphere_rigid_body_a.transform, &sphere_collider_b, &sphere_rigid_body_b.transform) {
                collision_data.solve(&mut sphere_rigid_body_a, &mut sphere_rigid_body_b);
            }
            if let Some(collision_data) = sphere_collider_a.collides(&sphere_rigid_body_a.transform, &plane_collider, &plane_rigid_body.transform) {
                collision_data.solve(&mut sphere_rigid_body_a, &mut plane_rigid_body);
            }
            if let Some(collision_data) = sphere_collider_b.collides(&sphere_rigid_body_b.transform, &plane_collider, &plane_rigid_body.transform) {
                collision_data.solve(&mut sphere_rigid_body_b, &mut plane_rigid_body);
            }

            sphere_rigid_body_a.force += gravity * (1.0 / sphere_rigid_body_a.inverse_mass);
            sphere_rigid_body_b.force += gravity * (1.0 / sphere_rigid_body_b.inverse_mass);

            sphere_rigid_body_a.update(DELTA_TIME);
            sphere_rigid_body_b.update(DELTA_TIME);

            accumulated_time -= DELTA_TIME;
        }

        sphere_gm_a.set_transformation(Matrix4::new(
            sphere_rigid_body_a.transform.m11 as f32, sphere_rigid_body_a.transform.m21 as f32, sphere_rigid_body_a.transform.m31 as f32, sphere_rigid_body_a.transform.m41 as f32,
            sphere_rigid_body_a.transform.m12 as f32, sphere_rigid_body_a.transform.m22 as f32, sphere_rigid_body_a.transform.m32 as f32, sphere_rigid_body_a.transform.m42 as f32,
            sphere_rigid_body_a.transform.m13 as f32, sphere_rigid_body_a.transform.m23 as f32, sphere_rigid_body_a.transform.m33 as f32, sphere_rigid_body_a.transform.m43 as f32,
            sphere_rigid_body_a.transform.m14 as f32, sphere_rigid_body_a.transform.m24 as f32, sphere_rigid_body_a.transform.m34 as f32, sphere_rigid_body_a.transform.m44 as f32,
        ) * Matrix4::from_scale(sphere_collider_a.radius as f32));
        sphere_gm_b.set_transformation(Matrix4::new(
            sphere_rigid_body_b.transform.m11 as f32, sphere_rigid_body_b.transform.m21 as f32, sphere_rigid_body_b.transform.m31 as f32, sphere_rigid_body_b.transform.m41 as f32,
            sphere_rigid_body_b.transform.m12 as f32, sphere_rigid_body_b.transform.m22 as f32, sphere_rigid_body_b.transform.m32 as f32, sphere_rigid_body_b.transform.m42 as f32,
            sphere_rigid_body_b.transform.m13 as f32, sphere_rigid_body_b.transform.m23 as f32, sphere_rigid_body_b.transform.m33 as f32, sphere_rigid_body_b.transform.m43 as f32,
            sphere_rigid_body_b.transform.m14 as f32, sphere_rigid_body_b.transform.m24 as f32, sphere_rigid_body_b.transform.m34 as f32, sphere_rigid_body_b.transform.m44 as f32,
        ) * Matrix4::from_scale(sphere_collider_b.radius as f32));

        sun.generate_shadow_map(1024, sphere_gm_a.into_iter().chain(&sphere_gm_b).chain(&plane_gm));

        frame_input.screen()
            .clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0))
            .render(&camera, sphere_gm_a.into_iter().chain(&sphere_gm_b).chain(&plane_gm), &[&sun, &ambient]);

        FrameOutput::default()
    });
}