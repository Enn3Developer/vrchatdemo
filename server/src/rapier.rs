use nalgebra::{Isometry3, SMatrix, Vector3};
use once_cell::sync::Lazy;
use rapier3d::control::{
    CharacterLength, EffectiveCharacterMovement, KinematicCharacterController,
};
use rapier3d::dynamics::MultibodyJointSet;
use rapier3d::math::Vector;
use rapier3d::pipeline::QueryFilter;
use rapier3d::prelude::{
    vector, CCDSolver, Capsule, ColliderSet, DefaultBroadPhase, ImpulseJointSet,
    IntegrationParameters, IslandManager, NarrowPhase, PhysicsPipeline, QueryPipeline,
    RigidBodySet,
};
use std::ops::{Deref, DerefMut};
use std::sync::Mutex;

pub static COLLIDER_SET: Lazy<Mutex<ColliderSet>> = Lazy::new(|| Mutex::new(ColliderSet::new()));
pub static RIGID_BODY_SET: Lazy<Mutex<RigidBodySet>> =
    Lazy::new(|| Mutex::new(RigidBodySet::new()));
pub static GRAVITY: Lazy<SMatrix<f32, 3, 1>> = Lazy::new(|| vector![0.0, 0.0, 0.0]);
pub static INTEGRATION_PARAMETERS: Lazy<IntegrationParameters> =
    Lazy::new(|| IntegrationParameters::default());
pub static PHYSICS_PIPELINE: Lazy<Mutex<PhysicsPipeline>> =
    Lazy::new(|| Mutex::new(PhysicsPipeline::new()));
pub static ISLAND_MANAGER: Lazy<Mutex<IslandManager>> =
    Lazy::new(|| Mutex::new(IslandManager::new()));
pub static BROAD_PHASE: Lazy<Mutex<DefaultBroadPhase>> =
    Lazy::new(|| Mutex::new(DefaultBroadPhase::new()));
pub static NARROW_PHASE: Lazy<Mutex<NarrowPhase>> = Lazy::new(|| Mutex::new(NarrowPhase::new()));
pub static IMPULSE_JOINT_SET: Lazy<Mutex<ImpulseJointSet>> =
    Lazy::new(|| Mutex::new(ImpulseJointSet::new()));
pub static MULTIBODY_JOINT_SET: Lazy<Mutex<MultibodyJointSet>> =
    Lazy::new(|| Mutex::new(MultibodyJointSet::new()));
pub static CCD_SOLVER: Lazy<Mutex<CCDSolver>> = Lazy::new(|| Mutex::new(CCDSolver::new()));
pub static QUERY_PIPELINE: Lazy<Mutex<QueryPipeline>> =
    Lazy::new(|| Mutex::new(QueryPipeline::new()));
pub static PLAYER_SHAPE: Lazy<Capsule> = Lazy::new(|| Capsule::new_y(0.5, 0.4));

pub struct ColliderData<V, I> {
    pub vertices: Vec<V>,
    pub indices: Vec<I>,
}

/// Default character controller
pub fn default_character_controller() -> KinematicCharacterController {
    let mut character_controller = KinematicCharacterController::default();
    character_controller.up = Vector::y_axis();
    character_controller.offset = CharacterLength::Absolute(0.01);
    // Don’t allow climbing slopes larger than 45 degrees
    character_controller.max_slope_climb_angle = 45_f32.to_radians();
    // Automatically slide down on slopes smaller than 30 degrees
    character_controller.min_slope_slide_angle = 30_f32.to_radians();
    // Snap to the ground if the vertical distance to the ground is smaller than 0.2
    character_controller.snap_to_ground = Some(CharacterLength::Absolute(0.2));

    character_controller
}

/// Computes the next physics step
pub fn physics_step() {
    PHYSICS_PIPELINE
        .lock()
        .expect("can't lock PHYSICS_PIPELINE")
        .step(
            GRAVITY.deref(),
            INTEGRATION_PARAMETERS.deref(),
            ISLAND_MANAGER
                .lock()
                .expect("can't lock ISLAND_MANAGER")
                .deref_mut(),
            BROAD_PHASE
                .lock()
                .expect("can't lock BROAD_PHASE")
                .deref_mut(),
            NARROW_PHASE
                .lock()
                .expect("can't lock NARROW_PHASE")
                .deref_mut(),
            RIGID_BODY_SET
                .lock()
                .expect("can't lock RIGID_BODY_SET")
                .deref_mut(),
            COLLIDER_SET
                .lock()
                .expect("can't lock COLLIDER_SET")
                .deref_mut(),
            IMPULSE_JOINT_SET
                .lock()
                .expect("can't lock IMPULSE_JOINT_SET")
                .deref_mut(),
            MULTIBODY_JOINT_SET
                .lock()
                .expect("can't lock MULTIBODY_JOINT_SET")
                .deref_mut(),
            CCD_SOLVER
                .lock()
                .expect("can't lock CCD_SOLVER")
                .deref_mut(),
            Some(
                QUERY_PIPELINE
                    .lock()
                    .expect("can't lock QUERY_PIPELINE")
                    .deref_mut(),
            ),
            &(),
            &(),
        );
}

/// Move a character controller starting from `pos` by `translation`
pub fn move_shape(
    character_controller: KinematicCharacterController,
    delta: f32,
    pos: &Isometry3<f32>,
    translation: Vector3<f32>,
) -> EffectiveCharacterMovement {
    character_controller.move_shape(
        delta,
        RIGID_BODY_SET
            .lock()
            .expect("can't lock RIGID_BODY_SET")
            .deref(),
        COLLIDER_SET
            .lock()
            .expect("can't lock COLLIDER_SET")
            .deref(),
        QUERY_PIPELINE
            .lock()
            .expect("can't lock QUERY_PIPELINE")
            .deref(),
        PLAYER_SHAPE.deref(),
        pos,
        translation,
        QueryFilter::exclude_dynamic(),
        |_collision| {},
    )
}

#[macro_export]
macro_rules! import_collider_data {
    () => {
        include!(concat!(env!("OUT_DIR"), "/collider_data.rs"));
    };
}
