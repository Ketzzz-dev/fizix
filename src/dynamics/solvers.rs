use crate::aliases::Vector3;
use crate::collisions::narrow_phase::CollisionManifold;
use crate::dynamics::RigidBody;

pub const NORM_EPSILON: f64 = 0.0001;

pub const RESTITUTION: f64 = 0.4;
pub const STATIC_FRICTION: f64 = 0.5;
pub const DYNAMIC_FRICTION: f64 = 0.3;

pub struct ResolutionData {
    pub linear_impulse: Vector3,
    pub angular_impulse_a: Vector3,
    pub angular_impulse_b: Vector3,

    pub correction: Vector3
}

pub fn resolve_collision(
    collision_manifold: &CollisionManifold,
    body_a: &RigidBody,
    body_b: &RigidBody
) -> ResolutionData {
    let linear_component = body_a.inverse_mass + body_b.inverse_mass;

    let mut linear_impulse = Vector3::zeros();
    let mut angular_impulse_a = Vector3::zeros();
    let mut angular_impulse_b = Vector3::zeros();

    let inverse_contact_count = 1.0 / collision_manifold.contacts.len() as f64;

    for contact in collision_manifold.contacts.iter() {
        let relative_contact_a = contact - body_a.position;
        let relative_contact_b = contact - body_b.position;

        let velocity_a = body_a.linear_velocity
            + body_a.angular_velocity.cross(&relative_contact_a);
        let velocity_b = body_b.linear_velocity
            + body_b.angular_velocity.cross(&relative_contact_b);

        let relative_velocity = velocity_a - velocity_b;
        let contact_velocity = relative_velocity.dot(&collision_manifold.normal);

        if contact_velocity > 0.0 { continue; }

        let rotation_per_unit_impulse_a = body_a.inverse_inertia_tensor_world
            * relative_contact_a.cross(&collision_manifold.normal);
        let rotation_per_unit_impulse_b = body_b.inverse_inertia_tensor_world
            * relative_contact_b.cross(&collision_manifold.normal);

        let velocity_per_unit_impulse_a = rotation_per_unit_impulse_a.cross(&relative_contact_a);
        let velocity_per_unit_impulse_b = rotation_per_unit_impulse_b.cross(&relative_contact_b);

        let angular_component = (velocity_per_unit_impulse_a + velocity_per_unit_impulse_b).dot(&collision_manifold.normal);
        let denominator = linear_component + angular_component;

        let reaction_scalar = -(1.0 + RESTITUTION) * contact_velocity / denominator;
        let reaction_impulse = collision_manifold.normal * reaction_scalar;

        linear_impulse += reaction_impulse * inverse_contact_count;
        angular_impulse_a += relative_contact_a.cross(&reaction_impulse) * inverse_contact_count;
        angular_impulse_b += relative_contact_b.cross(&reaction_impulse) * inverse_contact_count;

        let mut contact_tangent = relative_velocity - collision_manifold.normal * contact_velocity;

        if contact_tangent.norm_squared() <= NORM_EPSILON { continue; }

        contact_tangent.normalize_mut();

        let friction_scalar = -relative_velocity.dot(&contact_tangent) / denominator;
        let friction_impulse = if friction_scalar.abs() < reaction_scalar * STATIC_FRICTION {
            contact_tangent * friction_scalar
        } else {
            contact_tangent * -reaction_scalar * DYNAMIC_FRICTION
        };

        linear_impulse += friction_impulse * inverse_contact_count;
        angular_impulse_a += relative_contact_a.cross(&friction_impulse) * inverse_contact_count;
        angular_impulse_b += relative_contact_b.cross(&friction_impulse) * inverse_contact_count;
    }

    let correction = collision_manifold.penetration / linear_component * collision_manifold.normal;

    ResolutionData {
        linear_impulse,
        angular_impulse_a,
        angular_impulse_b,
        correction
    }
}