use nalgebra::{Point3, Quaternion, Vector3};
use three_d::{WindowSettings, Window, Srgba, PhysicalMaterial, Mesh, Gm, FrameOutput, DirectionalLight, CpuMesh, CpuMaterial, ClearState, Camera, vec3, degrees, AmbientLight, Matrix4};
use crate::collisions::{Collider, Cuboid, Plane, Sphere};
use crate::dynamics::{DynamicRigidBodySettings, RigidBody, StaticRigidBodySettings};
use crate::free_cam_control::FreeCamControl;

pub mod collisions;
pub mod dynamics;

mod free_cam_control;

const DELTA_TIME: f64 = 1.0 / 90.0;
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
        vec3(5.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(90.0),
        0.1,
        1000.0,
    );
    let mut controls = FreeCamControl::new(5.0, 2.5, 10.0);

    let mut plane_gm = Gm::new(
        Mesh::new(&context, &CpuMesh::square()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::new_opaque(255, 255, 255),

            ..Default::default()
        })
    );
    let mut sphere_gm = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::new(0, 0, 255, 128),

            ..Default::default()
        })
    );
    let mut cuboid_gm = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::new(0, 255, 0, 128),

            ..Default::default()
        })
    );

    plane_gm.set_transformation(
        Matrix4::from_translation(vec3(0.0, 0.0, 0.0)) *
            Matrix4::from_angle_x(degrees(90.0)) *
            Matrix4::from_scale(100.0)
    );

    let mut sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let mut plane_body = RigidBody::static_body(StaticRigidBodySettings::default(), Plane { normal: Vector3::new(0.0, 1.0, 0.0), offset: 0.0 });
    let mut sphere_body = RigidBody::dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(0.0, 10.0, 0.0),
        linear_velocity: Vector3::new(20.0, 2.0, 0.0),

        mass: 100.0,

        ..Default::default()
    }, Sphere { radius: 1.0 });
    let mut cuboid_body = RigidBody::dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(20.0, 8.0, 0.0),
        orientation: Quaternion::from_parts(0.0, Vector3::new(0.0, 0.0, 1.0)),

        mass: 20.0,

        ..Default::default()
    }, Cuboid { half_extents: Vector3::new(1.0, 7.0, 10.0) });

    let gravity = Vector3::new(0.0, -9.81, 0.0);

    let mut accumulated_time = 0.0;

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        while accumulated_time > DELTA_TIME {
            controls.update(&mut camera, DELTA_TIME);

            if let Some(collision_data) = plane_body.collider.collides(&plane_body.transform, &sphere_body.collider, &sphere_body.transform) {
                collision_data.solve(&mut plane_body, &mut sphere_body);
            }
            if let Some(collision_data) = plane_body.collider.collides(&plane_body.transform, &cuboid_body.collider, &cuboid_body.transform) {
                collision_data.solve(&mut plane_body, &mut cuboid_body);
            }
            if let Some(collision_data) = cuboid_body.collider.collides(&cuboid_body.transform, &sphere_body.collider, &sphere_body.transform) {
                collision_data.solve(&mut cuboid_body, &mut sphere_body);
            }

            sphere_body.force += gravity * (1.0 / sphere_body.inverse_mass);
            cuboid_body.force += gravity * (1.0 / cuboid_body.inverse_mass);

            sphere_body.update(TIME_SCALE * DELTA_TIME);
            cuboid_body.update(TIME_SCALE * DELTA_TIME);

            accumulated_time -= DELTA_TIME;
        }

        sphere_gm.set_transformation(Matrix4::new(
            sphere_body.transform.m11 as f32, sphere_body.transform.m21 as f32, sphere_body.transform.m31 as f32, sphere_body.transform.m41 as f32,
            sphere_body.transform.m12 as f32, sphere_body.transform.m22 as f32, sphere_body.transform.m32 as f32, sphere_body.transform.m42 as f32,
            sphere_body.transform.m13 as f32, sphere_body.transform.m23 as f32, sphere_body.transform.m33 as f32, sphere_body.transform.m43 as f32,
            sphere_body.transform.m14 as f32, sphere_body.transform.m24 as f32, sphere_body.transform.m34 as f32, sphere_body.transform.m44 as f32,
        ) * Matrix4::from_scale(sphere_body.collider.radius as f32));
        cuboid_gm.set_transformation(Matrix4::new(
            cuboid_body.transform.m11 as f32, cuboid_body.transform.m21 as f32, cuboid_body.transform.m31 as f32, cuboid_body.transform.m41 as f32,
            cuboid_body.transform.m12 as f32, cuboid_body.transform.m22 as f32, cuboid_body.transform.m32 as f32, cuboid_body.transform.m42 as f32,
            cuboid_body.transform.m13 as f32, cuboid_body.transform.m23 as f32, cuboid_body.transform.m33 as f32, cuboid_body.transform.m43 as f32,
            cuboid_body.transform.m14 as f32, cuboid_body.transform.m24 as f32, cuboid_body.transform.m34 as f32, cuboid_body.transform.m44 as f32,
        ) * Matrix4::from_nonuniform_scale(
            cuboid_body.collider.half_extents.x as f32,
            cuboid_body.collider.half_extents.y as f32,
            cuboid_body.collider.half_extents.z as f32
        ));

        sun.generate_shadow_map(1024, plane_gm.into_iter().chain(&sphere_gm).chain(&cuboid_gm));

        let render_target = frame_input.screen();

        render_target.clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0))
            .render(&camera, plane_gm.into_iter().chain(&sphere_gm).chain(&cuboid_gm), &[&sun, &ambient]);

        FrameOutput::default()
    });
}