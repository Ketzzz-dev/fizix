use crate::aliases::{Point3, TransformMatrix, Vector3};
use crate::collisions::Collider;

macro_rules! position {
    ($transform:expr) => {
        Point3::new($transform.m14, $transform.m24, $transform.m34)
    };
}
macro_rules! axes {
    ($transform:expr) => {
        [
            $transform.fixed_view::<3, 1>(0, 0).clone_owned(),
            $transform.fixed_view::<3, 1>(0, 1).clone_owned(),
            $transform.fixed_view::<3, 1>(0, 2).clone_owned()
        ]
    };
}
macro_rules! transform_vertices {
    ($vertices:expr, $transform:expr) => {
        $vertices.iter().map(|vertex| $transform.transform_point(vertex)).collect::<Vec<Point3>>()
    };
}
macro_rules! test_axis {
    (
        $axis:expr,
        $axis_type:expr,
        $vertices_a:expr,
        $vertices_b:expr,
        $min_overlap:expr,
        $best_axis:expr,
        $collision_axis:expr
    ) => {
        let (min_a, max_a) = project_to_axis(&$vertices_a, $axis);
        let (min_b, max_b) = project_to_axis(&$vertices_b, $axis);

        if max_a < min_b || max_b < min_a { return None; }

        let overlap = max_a.min(max_b) - min_a.max(min_b);

        if overlap < $min_overlap {
            $min_overlap = overlap;
            $best_axis = *$axis;
            $collision_axis = $axis_type;
        }
    };
}

pub const EPSILON: f64 = 1e-3;

pub struct CollisionManifold {
    pub contacts: Vec<Point3>,
    pub normal: Vector3,
    pub penetration: f64
}

pub enum AxisCollision {
    FaceA,
    FaceB,
    Edges
}

pub fn test_collision(
    collider_a: &Collider,
    transform_a: &TransformMatrix,
    collider_b: &Collider,
    transform_b: &TransformMatrix
) -> Option<CollisionManifold> {
    match (collider_a, collider_b) {
        (Collider::Sphere { radius: radius_a }, Collider::Sphere { radius: radius_b }) => {
            sphere_vs_sphere(*radius_a, transform_a, *radius_b, transform_b)
        }
        (Collider::Sphere { radius }, Collider::Plane { normal, offset }) => {
            sphere_vs_plane(*radius, transform_a, *normal, *offset)
        }
        (Collider::Plane { normal, offset }, Collider::Sphere { radius }) => {
            let mut collision_manifold = sphere_vs_plane(*radius, transform_b, *normal, *offset)?;

            collision_manifold.normal.neg_mut();

            Some(collision_manifold)
        }
        (Collider::Sphere { radius }, Collider::Cuboid { half_extents, .. }) => {
            sphere_vs_cuboid(*radius, transform_a, *half_extents, transform_b)
        }
        (Collider::Cuboid { half_extents, .. }, Collider::Sphere { radius }) => {
            let mut collision_manifold = sphere_vs_cuboid(
                *radius, transform_b,
                *half_extents, transform_a
            )?;

            collision_manifold.normal.neg_mut();

            Some(collision_manifold)
        }
        (Collider::Cuboid { local_vertices: vertices, .. }, Collider::Plane { normal, offset }) => {
            cuboid_vs_plane(vertices, transform_a, *normal, *offset)
        }
        (Collider::Plane { normal, offset }, Collider::Cuboid { local_vertices: vertices, .. }) => {
            let mut collision_manifold = cuboid_vs_plane(vertices, transform_b, *normal, *offset)?;

            collision_manifold.normal.neg_mut();

            Some(collision_manifold)
        },
        (Collider::Cuboid {
            local_vertices: local_vertices_a,
            ..
        }, Collider::Cuboid {
            local_vertices: local_vertices_b,
            ..
        }) => {
            cuboid_vs_cuboid(
                local_vertices_a, transform_a,
                local_vertices_b, transform_b
            )
        }
        _ => None
    }
}

pub fn sphere_vs_sphere(
    radius_a: f64,
    transform_a: &TransformMatrix,
    radius_b: f64,
    transform_b: &TransformMatrix
) -> Option<CollisionManifold> {
    let center_a = position!(transform_a);
    let center_b = position!(transform_b);

    let delta = center_a - center_b;
    let distance_squared = delta.norm_squared();
    let radii_sum = radius_a + radius_b;

    if distance_squared > radii_sum * radii_sum {
        return None;
    }

    let distance = distance_squared.sqrt();
    let normal = delta / distance;

    Some(CollisionManifold {
        contacts: vec![(center_a + normal * radius_a).into()],
        normal,
        penetration: radii_sum - distance
    })
}

pub fn sphere_vs_plane(
    sphere_radius: f64,
    sphere_transform: &TransformMatrix,
    plane_normal: Vector3,
    plane_offset: f64
) -> Option<CollisionManifold> {
    let sphere_center = position!(sphere_transform);
    let distance = sphere_center.coords.dot(&plane_normal) - plane_offset;

    if distance > sphere_radius {
        return None;
    }

    Some(CollisionManifold {
        contacts: vec![sphere_center - plane_normal * distance],
        normal: plane_normal,
        penetration: sphere_radius - distance
    })
}

pub fn sphere_vs_cuboid(
    sphere_radius: f64,
    sphere_transform: &TransformMatrix,
    cuboid_half_extents: Vector3,
    cuboid_transform: &TransformMatrix
) -> Option<CollisionManifold> {
    let sphere_center = position!(sphere_transform);
    let relative_center = cuboid_transform.try_inverse()?.transform_point(&sphere_center);

    if relative_center.x.abs() - sphere_radius > cuboid_half_extents.x
        || relative_center.y.abs() - sphere_radius > cuboid_half_extents.y
        || relative_center.z.abs() - sphere_radius > cuboid_half_extents.z
    { return None; }

    let closest_point = Point3::new(
        relative_center.x.clamp(-cuboid_half_extents.x, cuboid_half_extents.x),
        relative_center.y.clamp(-cuboid_half_extents.y, cuboid_half_extents.y),
        relative_center.z.clamp(-cuboid_half_extents.z, cuboid_half_extents.z),
    );

    let distance_sq = (relative_center - closest_point).norm_squared();

    if distance_sq > sphere_radius * sphere_radius {
        return None;
    }

    let closest_point_world = cuboid_transform.transform_point(&closest_point);
    let distance = distance_sq.sqrt();

    Some(CollisionManifold {
        contacts: vec![closest_point_world],
        normal: (sphere_center - closest_point_world) / distance,
        penetration: sphere_radius - distance_sq
    })
}

pub fn cuboid_vs_plane(
    cuboid_local_vertices: &[Point3],
    cuboid_transform: &TransformMatrix,
    plane_normal: Vector3,
    plane_offset: f64
) -> Option<CollisionManifold> {
    let mut contacts = vec![];
    let mut max_depth = f64::NEG_INFINITY;

    for local_vertex in cuboid_local_vertices.iter() {
        let world_vertex = cuboid_transform.transform_point(&local_vertex);
        let distance = world_vertex.coords.dot(&plane_normal);

        if distance <= plane_offset {
            let depth = plane_offset - distance;

            contacts.push(world_vertex + plane_normal * depth);

            if depth > max_depth {
                max_depth = depth;
            }
        }
    }

    if contacts.is_empty() { return None }

    Some(CollisionManifold {
        contacts,
        normal: plane_normal,
        penetration: max_depth
    })
}

pub fn cuboid_vs_cuboid(
    local_vertices_a: &[Point3],
    transform_a: &TransformMatrix,
    local_vertices_b: &[Point3],
    transform_b: &TransformMatrix
) -> Option<CollisionManifold> {
    let axes_a = axes!(transform_a);
    let axes_b = axes!(transform_b);

    let world_vertices_a = transform_vertices!(local_vertices_a, transform_a);
    let world_vertices_b = transform_vertices!(local_vertices_b, transform_b);

    let mut min_overlap = f64::INFINITY;
    let mut best_axis = Vector3::zeros();
    let mut collision_axis = AxisCollision::FaceA;

    for axis in axes_a.iter() {
        test_axis!(axis, AxisCollision::FaceA, world_vertices_a, world_vertices_b, min_overlap, best_axis, collision_axis);
    }
    for axis in axes_b.iter() {
        test_axis!(axis, AxisCollision::FaceB, world_vertices_a, world_vertices_b, min_overlap, best_axis, collision_axis);
    }
    for a in axes_a.iter() {
        for b in axes_b.iter() {
            let cross = a.cross(b);

            if cross.norm_squared() <= EPSILON { continue; }

            let axis = cross.normalize();

            test_axis!(&axis, AxisCollision::Edges, world_vertices_a, world_vertices_b, min_overlap, best_axis, collision_axis);
        }
    }

    let position_a = position!(transform_a);
    let position_b = position!(transform_b);
    let direction = position_a - position_b;

    if direction.dot(&best_axis) < 0.0 {
        best_axis.neg_mut();
    }

    let mut contacts = vec![];

    match collision_axis {
        AxisCollision::FaceA => {
            let reference_face_normal = -best_axis;

            let mut min_dot = f64::INFINITY;
            let mut incident_face_vertices = (0, 0, 0, 0) ;

            for (i, normal) in Collider::CUBOID_FACE_NORMALS.iter().enumerate() {
                let normal = transform_b.transform_vector(normal);
                let dot = normal.dot(&reference_face_normal);

                if dot < min_dot {
                    min_dot = dot;
                    incident_face_vertices = Collider::CUBOID_FACE_VERTICES[i];
                }
            }

            let half_extents = world_vertices_a.first()?.coords;
            let mut clipped_vertices = vec![
                world_vertices_b[incident_face_vertices.0],
                world_vertices_b[incident_face_vertices.1],
                world_vertices_b[incident_face_vertices.2],
                world_vertices_b[incident_face_vertices.3]
            ];

            for axis in axes_a.iter() {
                if axis.dot(&reference_face_normal).abs() > EPSILON { continue; }

                let offset = half_extents.dot(axis);

                clipped_vertices = clip_points_to_plane(&clipped_vertices, *axis, offset);
                clipped_vertices = clip_points_to_plane(&clipped_vertices, -axis, offset);
            }

            let mut max_offset = f64::NEG_INFINITY;

            for vertex in world_vertices_a.iter() {
                let offset = vertex.coords.dot(&reference_face_normal);

                if offset > max_offset {
                    max_offset = offset;
                }
            }
            for vertex in clipped_vertices.iter() {
                let distance = vertex.coords.dot(&reference_face_normal);

                if distance <= max_offset {
                    let depth = max_offset - distance;

                    contacts.push(vertex + reference_face_normal * depth);
                }
            }
        }
        AxisCollision::FaceB => {
            let reference_face_normal = best_axis;

            let mut min_dot = f64::INFINITY;
            let mut incident_face_vertices = (0, 0, 0, 0) ;

            for (i, normal) in Collider::CUBOID_FACE_NORMALS.iter().enumerate() {
                let normal = transform_a.transform_vector(normal);
                let dot = normal.dot(&reference_face_normal);

                if dot < min_dot {
                    min_dot = dot;
                    incident_face_vertices = Collider::CUBOID_FACE_VERTICES[i];
                }
            }

            let half_extents = world_vertices_b.first()?.coords;
            let mut clipped_vertices = vec![
                world_vertices_a[incident_face_vertices.0],
                world_vertices_a[incident_face_vertices.1],
                world_vertices_a[incident_face_vertices.2],
                world_vertices_a[incident_face_vertices.3]
            ];

            for axis in axes_b.iter() {
                if axis.dot(&reference_face_normal).abs() > EPSILON { continue; }

                let offset = half_extents.dot(axis);

                clipped_vertices = clip_points_to_plane(&clipped_vertices, *axis, offset);
                clipped_vertices = clip_points_to_plane(&clipped_vertices, -axis, offset);
            }

            let mut max_offset = f64::NEG_INFINITY;

            for vertex in world_vertices_b.iter() {
                let offset = vertex.coords.dot(&reference_face_normal);

                if offset > max_offset {
                    max_offset = offset;
                }
            }
            for vertex in clipped_vertices.iter() {
                let distance = vertex.coords.dot(&reference_face_normal);

                if distance <= max_offset {
                    let depth = max_offset - distance;

                    contacts.push(vertex + reference_face_normal * depth);
                }
            }
        }
        AxisCollision::Edges => {
            let edge_a = get_support_edge(&world_vertices_a, -best_axis);
            let edge_b = get_support_edge(&world_vertices_b, best_axis);

            let d1 = edge_a.1 - edge_a.0;
            let d2 = edge_b.1 - edge_b.0;
            let r = edge_a.0 - edge_b.0;

            let a = d1.norm_squared();
            let e = d2.norm_squared();
            let f = d2.dot(&r);

            if a <= EPSILON && e <= EPSILON {
                contacts.push((0.5 * (edge_a.0.coords + edge_b.0.coords)).into());
            }

            let mut s;
            let mut t;

            if a <= EPSILON {
                s = 0.0;
                t = (f / e).clamp(0.0, 1.0);
            } else {
                let c = d1.dot(&r);

                if e <= EPSILON {
                    t = 0.0;
                    s = (-c / a).clamp(0.0, 1.0);
                } else {
                    let b = d1.dot(&d2);
                    let denom = a * e - b * b;

                    if denom != 0.0 {
                        s = ((b * f - c * e) / denom).clamp(0.0, 1.0);
                    } else {
                        s = 0.0;
                    }

                    let nom = b * s + f;

                    if nom < 0.0 {
                        t = 0.0;
                        s = (-c / a).clamp(0.0, 1.0);
                    } else if nom > e {
                        t = 1.0;
                        s = ((b - c) / a).clamp(0.0, 1.0);
                    } else {
                        t = nom / e;
                    }
                }
            }

            let c1 = edge_a.0 + s * d1;
            let c2 = edge_b.0 + t * d2;

            contacts.push((0.5 * (c1.coords + c2.coords)).into())
        }
    }

    Some(CollisionManifold {
        contacts,
        normal: best_axis,
        penetration: min_overlap
    })
}

fn project_to_axis(vertices: &[Point3], axis: &Vector3) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;

    for vertex in vertices.iter() {
        let projection = vertex.coords.dot(axis);

        if projection < min {
            min = projection;
        }
        if projection > max {
            max = projection;
        }
    }

    (min, max)
}

fn clip_points_to_plane(vertices: &[Point3], normal: Vector3, offset: f64) -> Vec<Point3> {
    let mut clipped_vertices = vec![];
    let mut last_vertex = &vertices[vertices.len() - 1];
    let mut last_distance = last_vertex.coords.dot(&normal) - offset;

    for current_vertex in vertices.iter() {
        let current_distance = current_vertex.coords.dot(&normal) - offset;

        if last_distance < 0.0 && current_distance < 0.0 {
            clipped_vertices.push(*current_vertex);
        } else if last_distance < 0.0 && current_distance > 0.0 {
            let t = last_distance / (last_distance - current_distance);

            clipped_vertices.push(*last_vertex + t * (current_vertex - last_vertex));
        } else if current_distance < 0.0 && last_distance > 0.0 {
            let t = last_distance / (last_distance - current_distance);

            clipped_vertices.push(*last_vertex + t * (current_vertex - last_vertex));
            clipped_vertices.push(*current_vertex);
        }

        last_vertex = current_vertex;
        last_distance = current_distance;
    }

    clipped_vertices
}

fn get_support_edge(vertices: &[Point3], normal: Vector3) -> (Point3, Point3) {
    let mut max_dot = f64::NEG_INFINITY;
    let mut support_edge = (Point3::origin(), Point3::origin());

    for (a, b) in Collider::CUBOID_EDGE_VERTICES.iter() {
        let a = &vertices[*a];
        let b = &vertices[*b];

        let mid_point = 0.5 * (a.coords + b.coords);
        let dot = mid_point.dot(&normal);

        if dot > max_dot {
            max_dot = dot;
            support_edge = (*a, *b);
        }
    }

    support_edge
}