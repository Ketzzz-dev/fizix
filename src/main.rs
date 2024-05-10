use std::f64::consts::PI;
use rand::{Rng, thread_rng};
use three_d::{
    degrees, vec3, AmbientLight, Camera, ClearState, CpuMaterial, CpuMesh, DirectionalLight,
    FrameOutput, Gm, Matrix4, Mesh, PhysicalMaterial, Srgba, Window, WindowSettings,
};
use crate::aliases::{Point3, Quaternion, Vector3};
use crate::collisions::Collider;
use crate::dynamics::{DynamicRigidBodySettings, StaticRigidBodySettings, World};

use crate::free_cam_control::FreeCamControl;

pub mod collisions;
pub mod dynamics;

mod free_cam_control;
mod aliases;

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
        vec3(0.0, 2.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(90.0),
        0.1,
        1000.0
    );
    let mut controls = FreeCamControl::new(5.0, 5.0, 10.0);

    let sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let mut world = World::new();
    let mut albedos = vec![];

    world.create_static_body(StaticRigidBodySettings::default(), Collider::plane(Vector3::new(0.0, 1.0, 0.0), -2.0));
    albedos.push(Srgba::new_opaque(255, 255, 255));

    world.create_dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(0.0, 1.0, 0.0),

        mass: 10.0,

        ..Default::default()
    }, Collider::cuboid(Vector3::new(3.0, 1.0, 3.0)));
    albedos.push(Srgba::new_opaque(0xf4, 0x73, 0x33));

    world.create_dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(0.0, 4.0, 0.0),
        orientation: euler_angles(0.0, PI / 4.0, 0.0),

        mass: 5.0,

        ..Default::default()
    }, Collider::cuboid(Vector3::new(3.0, 1.0, 4.0)));
    albedos.push(Srgba::new_opaque(0xf4, 0x73, 0x33));

    world.create_dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(0.0, 7.0, 0.0),

        ..Default::default()
    }, Collider::sphere(1.0));
    albedos.push(Srgba::new_opaque(0xf4, 0x73, 0x33));

    let mut accumulated_time = 0.0;
    let mut rng = thread_rng();

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        while accumulated_time > DELTA_TIME {
            controls.update(&mut camera, DELTA_TIME);
            world.update(DELTA_TIME * TIME_SCALE);

            accumulated_time -= DELTA_TIME;
        }

        let render_target = frame_input.screen();

        render_target.clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0));

        for i in 0..world.bodies.len() {
            let transform = convert_transform(&&world.bodies[i].transform_matrix);
            let scale = calculate_scale(&world.colliders[i]);

            let cpu_mesh = match &world.colliders[i] {
                Collider::Sphere { .. } => CpuMesh::sphere(16),
                Collider::Plane { .. } => CpuMesh::square(),
                Collider::Cuboid { .. } => CpuMesh::cube()
            };

            let mut gm = Gm::new(
                Mesh::new(&context, &cpu_mesh),
                PhysicalMaterial::new_opaque(
                    &context,
                    &CpuMaterial {
                        albedo: albedos[i],

                        ..Default::default()
                    },
                ),
            );

            gm.set_transformation(transform * scale);
            render_target.render(
                &camera,
                gm.into_iter(),
                &[&sun, &ambient]
            );
        }
        for (collision, _, _) in world.collisions.iter() {
            for (i, contact) in collision.contacts.iter().enumerate() {
                let t = i as f64 / collision.contacts.len() as f64;

                let mut point_gm = Gm::new(
                    Mesh::new(&context, &CpuMesh::sphere(8)),
                    PhysicalMaterial::new_opaque(
                        &context,
                        &CpuMaterial {
                            albedo: Srgba::new_opaque((t * 255.0) as u8, 0, ((1.0 - t) * 255.0) as u8),

                            ..Default::default()
                        },
                    ),
                );

                point_gm.set_transformation(Matrix4::from_translation(vec3(
                    contact.x as f32,
                    contact.y as f32,
                    contact.z as f32
                )) * Matrix4::from_scale(0.1));
                render_target.render(
                    &camera,
                    point_gm.into_iter(),
                    &[&sun, &ambient]
                );
            }
        }

        FrameOutput::default()
    });
}

pub fn convert_transform(transform: &nalgebra::Matrix4<f64>) -> Matrix4<f32> {
    Matrix4::new(
        transform.m11 as f32, transform.m21 as f32, transform.m31 as f32, transform.m41 as f32,
        transform.m12 as f32, transform.m22 as f32, transform.m32 as f32, transform.m42 as f32,
        transform.m13 as f32, transform.m23 as f32, transform.m33 as f32, transform.m43 as f32,
        transform.m14 as f32, transform.m24 as f32, transform.m34 as f32, transform.m44 as f32
    )
}

pub fn calculate_scale(collider: &Collider) -> Matrix4<f32> {
    match collider {
        Collider::Cuboid { half_extents, .. } => Matrix4::from_nonuniform_scale(half_extents.x as f32, half_extents.y as f32, half_extents.z as f32),
        Collider::Sphere { radius } => Matrix4::from_scale(*radius as f32),
        Collider::Plane { normal, offset } => {
            Matrix4::from_translation(vec3(normal.x as f32, normal.y as f32, normal.z as f32) * *offset as f32)
                * Matrix4::from_angle_x(degrees(90.0))
                * Matrix4::from_scale(100.0)
        }
    }
}

pub fn euler_angles(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
    let half_roll = roll * 0.5;
    let half_pitch = pitch * 0.5;
    let half_yaw = yaw * 0.5;

    let sin_roll = half_roll.sin();
    let cos_roll = half_roll.cos();
    let sin_pitch = half_pitch.sin();
    let cos_pitch = half_pitch.cos();
    let sin_yaw = half_yaw.sin();
    let cos_yaw = half_yaw.cos();

    Quaternion::new(
        cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
        sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
        cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
        cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw
    )
}