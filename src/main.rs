use nalgebra::{Point3, Vector3};
use three_d::{
    degrees, vec3, AmbientLight, Camera, ClearState, CpuMaterial, CpuMesh, DirectionalLight,
    FrameOutput, Gm, Matrix4, Mesh, PhysicalMaterial, Srgba, Window, WindowSettings,
};

use crate::collisions::{Cuboid, Plane, Sphere};
use crate::dynamics::{DynamicRigidBodySettings, StaticRigidBodySettings, World};
use crate::free_cam_control::FreeCamControl;

pub mod collisions;
pub mod dynamics;

mod free_cam_control;

const DELTA_TIME: f64 = 1.0 / 75.0;
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
        vec3(5.0, 2.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(90.0),
        0.1,
        100.0
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
    let mut sphere_gm = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                albedo: Srgba::new(0, 0, 255, 128),

                ..Default::default()
            },
        ),
    );
    let mut cuboid_gm = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_opaque(
            &context,
            &CpuMaterial {
                albedo: Srgba::new(0, 255, 0, 128),

                ..Default::default()
            },
        ),
    );

    plane_gm.set_transformation(
        Matrix4::from_translation(vec3(0.0, 0.0, 0.0))
            * Matrix4::from_angle_x(degrees(90.0))
            * Matrix4::from_scale(50.0)
    );

    let mut sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let mut world = World::new();

    world.create_static_body(StaticRigidBodySettings::default(), Plane {
        normal: Vector3::new(0.0, 1.0, 0.0),
        offset: 0.0
    });

    let sphere_index = world.create_dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(0.0, 0.0, 0.0),
        linear_velocity: Vector3::new(50.0, 25.0, 10.0),

        mass: 1.0,

        ..Default::default()
    }, Sphere { radius: 1.0 });
    let cuboid_index = world.create_dynamic_body(DynamicRigidBodySettings {
        position: Point3::new(20.0, 7.0, 0.0),

        mass: 10.0,

        ..Default::default()
    }, Cuboid::new(Vector3::new(1.0, 7.0, 10.0)));

    let mut accumulated_time = 0.0;

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        while accumulated_time > DELTA_TIME {
            controls.update(&mut camera, DELTA_TIME);
            world.update(DELTA_TIME * TIME_SCALE);

            accumulated_time -= DELTA_TIME;
        }

        let sphere_body = world.get_body(sphere_index);
        let sphere_collider = world.get_collider(sphere_index);

        if let (Some(sphere_body), Some(sphere_collider)) = (sphere_body, sphere_collider) {
            sphere_gm.set_transformation(
                convert_transform(&sphere_body.transform)
                    * sphere_collider.calculate_scale_matrix()
            );
        }

        let cuboid_body = world.get_body(cuboid_index);
        let cuboid_collider = world.get_collider(cuboid_index);

        if let (Some(cuboid_body), Some(cuboid_collider)) = (cuboid_body, cuboid_collider) {
            cuboid_gm.set_transformation(
                convert_transform(&cuboid_body.transform)
                    * cuboid_collider.calculate_scale_matrix()
            );
        }

        sun.generate_shadow_map(
            1024,
            plane_gm.into_iter().chain(&sphere_gm).chain(&cuboid_gm),
        );

        let render_target = frame_input.screen();

        render_target
            .clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0))
            .render(
                &camera,
                plane_gm.into_iter().chain(&sphere_gm).chain(&cuboid_gm),
                &[&sun, &ambient],
            );

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
