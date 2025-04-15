use nalgebra::SMatrix;
use once_cell::sync::Lazy;
use rapier3d::dynamics::MultibodyJointSet;
use rapier3d::prelude::{
    vector, CCDSolver, Capsule, ColliderSet, DefaultBroadPhase, ImpulseJointSet,
    IntegrationParameters, IslandManager, NarrowPhase, PhysicsPipeline, QueryPipeline,
    RigidBodySet,
};
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
