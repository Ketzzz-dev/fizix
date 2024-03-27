use three_d::{WindowSettings, Window, Srgba, PhysicalMaterial, Mesh, Gm, FrameOutput, DirectionalLight, CpuMesh, CpuMaterial, ClearState, Camera, vec3, degrees, AmbientLight, Matrix4};
use crate::collisions::{CollisionDetector, Plane, Sphere};

use crate::dynamics::{RigidBody, World};
use crate::free_cam_control::FreeCamControl;
use crate::math::{Vector3, Quaternion, Matrix4x4};

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

    let mut sphere_a = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::GREEN,

            ..Default::default()
        })
    );
    let mut sphere_b = Gm::new(
        Mesh::new(&context, &CpuMesh::sphere(16)),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::BLUE,

            ..Default::default()
        })
    );
    let mut plane = Gm::new(
        Mesh::new(&context, &CpuMesh::square()),
        PhysicalMaterial::new_opaque(&context, &CpuMaterial {
            albedo: Srgba::new_opaque(120, 100, 120),

            ..Default::default()
        })
    );

    plane.set_transformation(
        Matrix4::from_translation(vec3(0.0, -10.0, 0.0)) *
            Matrix4::from_angle_x(degrees(90.0)) *
            Matrix4::from_scale(25.0)
    );

    let mut sun = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -1.0, -0.707));
    let ambient = AmbientLight::new(&context, 0.05, Srgba::WHITE);

    let mut world = World::new();
    
    let body_a = RigidBody::new(-Vector3::X_AXIS * 1.5, Quaternion::IDENTITY, 10.0);
    let body_b = RigidBody::new(Vector3::X_AXIS * 1.5, Quaternion { w: 1.0, x: 1.0, y: 1.0, z: 0.0 }, 10.0);

    let primitive_a = Sphere { radius: 1.0 };
    let primitive_b = Sphere { radius: 1.0 };
    let primitive_c = Plane { normal: Vector3::Y_AXIS, offset: -10.0 };

    let body_index_a = world.add_body(body_a);
    let body_index_b = world.add_body(body_b);

    let mut accumulated_time = 0.0;

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        controls.handle_events(&mut camera, &mut frame_input.events);

        accumulated_time += 0.001 * frame_input.elapsed_time;

        while accumulated_time > DELTA_TIME {
            controls.update(&mut camera, DELTA_TIME);

            if let Some(body) = world.get_body_mut(body_index_a) {
                if let Some(collision_data) = CollisionDetector::sphere_plane(&primitive_a, &body, &primitive_c, &body) {
                    for contact in &collision_data.contacts {
                        let normal_velocity = body.linear_velocity.dot(&contact.normal);

                        if normal_velocity > 0.0 { break; }

                        let restitution = 0.707;
                        let impulse = -(1.0 + restitution) * normal_velocity;
                        let impulse_vector = impulse * contact.normal;

                        body.linear_velocity += impulse_vector;
                    }
                    for contact in &collision_data.contacts {
                        let move_per_mass = contact.normal * contact.penetration;

                        body.position += move_per_mass;
                    }
                }
            }
            if let Some(body) = world.get_body_mut(body_index_b) {
                if let Some(collision_data) = CollisionDetector::sphere_plane(&primitive_b, &body, &primitive_c, &body) {
                    for contact in &collision_data.contacts {
                        let normal_velocity = body.linear_velocity.dot(&contact.normal);

                        if normal_velocity > 0.0 { break; }

                        let restitution = 0.292;
                        let impulse = -(1.0 + restitution) * normal_velocity;
                        let impulse_vector = impulse * contact.normal;

                        body.linear_velocity += impulse_vector;
                    }
                    for contact in &collision_data.contacts {
                        let move_per_mass = contact.normal * contact.penetration;

                        body.position += move_per_mass;
                    }
                }
            }

            world.update(DELTA_TIME);

            accumulated_time -= DELTA_TIME;
        }

        if let Some(body_a) = world.get_body(body_index_a) {
            sphere_a.set_transformation(to_cg_matrix(&body_a.transform));
        }
        if let Some(body_b) = world.get_body(body_index_b) {
            sphere_b.set_transformation(to_cg_matrix(&body_b.transform));
        }

        sun.generate_shadow_map(1024, sphere_a.into_iter().chain(&sphere_b));

        frame_input.screen()
            .clear(ClearState::color_and_depth(0.043, 0.051, 0.067, 1.0, 1.0))
            .render(&camera, sphere_a.into_iter().chain(&sphere_b).chain(&plane), &[&sun, &ambient]);

        FrameOutput::default()
    });
}

fn to_cg_matrix(transform: &Matrix4x4) -> Matrix4<f32> {
    Matrix4::new(
        transform.m11 as f32, transform.m21 as f32, transform.m31 as f32, transform.m41 as f32,
        transform.m12 as f32, transform.m22 as f32, transform.m32 as f32, transform.m42 as f32,
        transform.m13 as f32, transform.m23 as f32, transform.m33 as f32, transform.m43 as f32,
        transform.m14 as f32, transform.m24 as f32, transform.m34 as f32, transform.m44 as f32
    )
}