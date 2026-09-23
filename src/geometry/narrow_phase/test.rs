//! Narrow-phase regression tests (collider re-parenting interactions).

#[allow(unused_imports)]
use crate::alloc_prelude::*;
use crate::math::Vector;
use crate::prelude::{
    CCDSolver, ColliderBuilder, DefaultBroadPhase, IntegrationParameters, PhysicsPipeline,
    RigidBodyBuilder,
};
use std::println;

use super::*;

use crate::dynamics::{ImpulseJointSet, MultibodyJointSet, SoftBodySet};

/// Test for https://github.com/dimforge/rapier/issues/734.
#[test]
pub fn collider_set_parent_depenetration() {
    // This tests the scenario:
    // 1. Body A has two colliders attached (and overlapping), Body B has none.
    // 2. One of the colliders from Body A gets re-parented to Body B.
    //    -> Collision is properly detected between the colliders of A and B.
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::ball(0.5);

    /* Create body 1, which will contain both colliders at first. */
    let rigid_body_1 = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.0, 0.0))
        .build();
    let body_1_handle = rigid_body_set.insert(rigid_body_1);

    /* Create collider 1. Parent it to rigid body 1. */
    let collider_1_handle =
        collider_set.insert_with_parent(collider.build(), body_1_handle, &mut rigid_body_set);

    /* Create collider 2. Parent it to rigid body 1. */
    let collider_2_handle =
        collider_set.insert_with_parent(collider.build(), body_1_handle, &mut rigid_body_set);

    /* Create body 2. No attached colliders yet. */
    let rigid_body_2 = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.0, 0.0))
        .build();
    let body_2_handle = rigid_body_set.insert(rigid_body_2);

    /* Create other structures necessary for the simulation. */
    let gravity = Vector::ZERO;
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut soft_body_set = SoftBodySet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );
    let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
    let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
    assert!((collider_1_position.translation - collider_2_position.translation).length() < 0.5f32);

    // Same-parent pairs are filtered out by the broad phase (issue #970), so no (empty)
    // contact pair is registered while both colliders share their parent. If one of them
    // is re-parented, the broad phase must re-generate the pair (asserted below).
    assert!(
        narrow_phase
            .contact_pair(collider_1_handle, collider_2_handle)
            .is_none_or(|pair| pair.manifolds().is_empty()),
        "No contact should be simulated between same-parent colliders."
    );
    assert!(
        narrow_phase
            .intersection_pair(collider_1_handle, collider_2_handle)
            .is_none(),
        "Interaction pair is for sensors"
    );
    /* Parent collider 2 to body 2. */
    collider_set.set_parent(collider_2_handle, Some(body_2_handle), &mut rigid_body_set);

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(contact_pair.manifolds().len(), 1);
    assert!(
        narrow_phase
            .intersection_pair(collider_1_handle, collider_2_handle)
            .is_none(),
        "Interaction pair is for sensors"
    );

    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..200 {
        physics_pipeline.step(
            gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut soft_body_set,
            &mut ccd_solver,
            &physics_hooks,
            &event_handler,
        );

        let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
        let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
        println!("collider 1 position: {}", collider_1_position.translation);
        println!("collider 2 position: {}", collider_2_position.translation);
    }

    let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
    let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
    println!("collider 2 position: {}", collider_2_position.translation);
    assert!(
        (collider_1_position.translation - collider_2_position.translation).length() >= 0.5f32,
        "colliders should no longer be penetrating."
    );
}

/// Test for https://github.com/dimforge/rapier/issues/734.
#[test]
pub fn collider_set_parent_no_self_intersection() {
    // This tests the scenario:
    // 1. Body A and Body B each have one collider attached.
    //    -> There should be a collision detected between A and B.
    // 2. The collider from Body B gets attached to Body A.
    //    -> There should no longer be any collision between A and B.
    // 3. Re-parent one of the collider from Body A to Body B again.
    //    -> There should a collision again.
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::ball(0.5);

    /* Create body 1, which will contain collider 1. */
    let rigid_body_1 = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.0, 0.0))
        .build();
    let body_1_handle = rigid_body_set.insert(rigid_body_1);

    /* Create collider 1. Parent it to rigid body 1. */
    let collider_1_handle =
        collider_set.insert_with_parent(collider.build(), body_1_handle, &mut rigid_body_set);

    /* Create body 2, which will contain collider 2 at first. */
    let rigid_body_2 = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.0, 0.0))
        .build();
    let body_2_handle = rigid_body_set.insert(rigid_body_2);

    /* Create collider 2. Parent it to rigid body 2. */
    let collider_2_handle =
        collider_set.insert_with_parent(collider.build(), body_2_handle, &mut rigid_body_set);

    /* Create other structures necessary for the simulation. */
    let gravity = Vector::ZERO;
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut soft_body_set = SoftBodySet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(
        contact_pair.manifolds().len(),
        1,
        "There should be a contact manifold."
    );

    let collider_1_position = collider_set.get(collider_1_handle).unwrap().pos;
    let collider_2_position = collider_set.get(collider_2_handle).unwrap().pos;
    assert!((collider_1_position.translation - collider_2_position.translation).length() < 0.5f32);

    /* Parent collider 2 to body 1. */
    collider_set.set_parent(collider_2_handle, Some(body_1_handle), &mut rigid_body_set);
    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should no longer exist.");
    assert_eq!(
        contact_pair.manifolds().len(),
        0,
        "Colliders with same parent should not be in contact together."
    );

    /* Parent collider 2 back to body 1. */
    collider_set.set_parent(collider_2_handle, Some(body_2_handle), &mut rigid_body_set);
    physics_pipeline.step(
        gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut soft_body_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    let contact_pair = narrow_phase
        .contact_pair(collider_1_handle, collider_2_handle)
        .expect("The contact pair should exist.");
    assert_eq!(
        contact_pair.manifolds().len(),
        1,
        "There should be a contact manifold."
    );
}

/// Balls resting on a voxels shape. parry's voxels–ball contact generator
/// emits one manifold per touched voxel without a persistent workspace, so
/// the pair's manifold ordinals change from step to step. Before voxels
/// pairs were treated as composite, the incremental solver-contact graph
/// kept stale ordinals: a debug build tripped "solver contact graph size !=
/// selection size", and every build panicked with "stale ContactRef
/// manifold ordinal" within a few hundred steps.
#[test]
pub fn balls_resting_on_voxels_keep_the_solver_graph_consistent() {
    use crate::math::IVector;
    use crate::pipeline::PhysicsWorld;

    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = 1.0 / 72.0;
    // A 6 m square of 0.1 m voxels, uneven by up to two cells.
    let mut cells = Vec::new();
    for x in 0..60 {
        for z in 0..60 {
            for y in -3..=((x * 7 + z * 13) % 5) / 2 {
                cells.push(IVector::new(x, y, z));
            }
        }
    }
    world.insert_collider(
        ColliderBuilder::voxels(Vector::splat(0.1), &cells).build(),
        None,
    );
    for i in 0..20 {
        let (a, b) = ((i * 37 % 50) as f32 * 0.1, (i * 53 % 50) as f32 * 0.1);
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(0.5 + a, 0.6, 0.5 + b)),
            ColliderBuilder::ball(0.12),
        );
    }
    for _ in 0..600 {
        world.step();
    }
    // Every ball came to rest on the voxels rather than falling through.
    for (_, body) in world.rigid_bodies() {
        assert!(body.translation().y > 0.0, "a ball fell through the voxels");
    }
}
