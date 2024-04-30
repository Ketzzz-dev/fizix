use three_d::{
    degrees, vec3, AmbientLight, Camera, ClearState, CpuMaterial, CpuMesh, DirectionalLight,
    FrameOutput, Gm, Matrix4, Mesh, PhysicalMaterial, Srgba, Window, WindowSettings,
};
use crate::aliases::{Point3, Vector3};
use crate::collisions::Collider;
use crate::dynamics::{DynamicRigidBodySettings, StaticRigidBodySettings, World};

use crate::free_cam_control::FreeCamControl;

pub mod collisions;
pub mod dynamics;

mod free_cam_control;
mod aliases;

const DELTA_TIME: f64 = 1.0 / 75.0;
const TIME_SCALE: f64 = 1.0 / 10.0;

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

    let mut plane_gm = Gm::new(
        Mesh::new(&context, &CpuMesh::square()),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                albedo: Srgba::new_opaque(255, 255, 255),

                ..Default::default()
            },
        ),
    );
    let mut gm_a = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new(
            &context,
            &CpuMaterial {
                albedo: Srgba::new(0, 0, 255, 128),

                ..Default::default()
            },
        ),
    );
    let mut gm_b = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new(
            &context,
            &CpuMaterial {
                albedo: Srgba::new(0, 255, 0, 128),

                ..Default::default()
            },
        ),
    );
    let mut gm_c = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                albedo: Srgba::new(0, 255, 255, 128),

                ..Default::default()
            },
        ),
    );

    plane_gm.set_transformation(
        Matrix4::from_translation(vec3(0.0, -5.0, 0.0))
            * Matrix4::from_angle_x(degrees(90.0))
            * Matrix4::from_scale(50.0)
    );

    let mut sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let mut world = World::new();

    world.create_static_body(StaticRigidBodySettings::default(), Collider::plane(Vector3::new(0.0, 1.0, 0.0), -5.0));

    let body_a = world.create_dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(0.0, 3.0, 0.0),

        linear_velocity: Vector3::new(7.5, 0.0, 0.0),
        angular_velocity: Vector3::new(0.0, 0.0, 1.0),

        mass: 5.0,

        ..Default::default()
    }, Collider::cuboid(Vector3::new(2.0, 1.0, 1.0)));
    let body_b = world.create_dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(20.0, 3.0, 0.0),

        linear_velocity: Vector3::new(-7.5, 0.0, 0.0),
        angular_velocity: Vector3::new(0.0, -1.0, 0.0),

        mass: 5.0,

        ..Default::default()
    }, Collider::cuboid(Vector3::new(2.0, 1.0, 2.0)));
    let body_c = world.create_dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(0.0, 3.0, 2.0),

        linear_velocity: Vector3::new(5.0, 3.0, 0.5),
        angular_velocity: Vector3::new(0.0, 2.0, 0.0),

        mass: 10.0,

        ..Default::default()
    }, Collider::sphere(1.0));

    let mut accumulated_time = 0.0;

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        let mut contacts: Vec<Point3> = vec![];

        while accumulated_time > DELTA_TIME {
            controls.update(&mut camera, DELTA_TIME);
            world.update(DELTA_TIME * TIME_SCALE);

            for (manifold, _, _) in world.collisions.iter() {
                for contact in manifold.contacts.iter() {
                    contacts.push(*contact);
                }
            }

            accumulated_time -= DELTA_TIME;
        }

        if let (Some(body), Some(collider)) = (world.get_body(body_a), world.get_collider(body_a)) {
            gm_a.set_transformation(
                convert_transform(&body.transform_matrix) * calculate_scale(collider)
            );
        }
        if let (Some(body), Some(collider)) = (world.get_body(body_b), world.get_collider(body_b)) {
            gm_b.set_transformation(
                convert_transform(&body.transform_matrix) * calculate_scale(collider)
            );
        }
        if let (Some(body), Some(collider)) = (world.get_body(body_c), world.get_collider(body_c)) {
            gm_c.set_transformation(
                convert_transform(&body.transform_matrix) * calculate_scale(collider)
            );
        }

        sun.generate_shadow_map(
            1024,
            plane_gm.into_iter().chain(&gm_a).chain(&gm_b).chain(&gm_c),
        );

        let render_target = frame_input.screen();

        render_target
            .clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0))
            .render(
                &camera,
                plane_gm.into_iter().chain(&gm_a).chain(&gm_b).chain(&gm_c),
                &[&sun, &ambient],
            );

        for contact in contacts.iter() {
            let mut gm = Gm::new(
                Mesh::new(&context, &CpuMesh::sphere(16)),
                PhysicalMaterial::new_opaque(
                    &context,
                    &CpuMaterial {
                        albedo: Srgba::new(255, 0, 0, 255),

                        ..Default::default()
                    },
                ),
            );

            gm.set_transformation(
                Matrix4::from_translation(vec3(contact.x as f32, contact.y as f32, contact.z as f32))
                    * Matrix4::from_scale(0.1)
            );

            render_target.render(&camera, gm.into_iter(), &[&sun, &ambient]);
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
        Collider::Plane { .. } => { unreachable!() }
    }
}