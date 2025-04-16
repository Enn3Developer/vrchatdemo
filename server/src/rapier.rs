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
pub type VectorTuple = (f32, f32, f32);
pub type VectorArray<T> = [T; 3];
pub type NormalColliderData = ColliderData<VectorTuple, VectorArray<u32>>;

pub trait FromBytes {
    type Output;

    fn from_bytes(bytes: &[u8], index: &mut usize) -> Self::Output;
}

impl FromBytes for [u32; 3] {
    type Output = [u32; 3];

    fn from_bytes(bytes: &[u8], index: &mut usize) -> Self::Output {
        let mut data = [0, 0, 0];

        for i in 0..3 {
            let bytes = [
                bytes[*index],
                bytes[*index + 1],
                bytes[*index + 2],
                bytes[*index + 3],
            ];
            data[i] = u32::from_le_bytes(bytes);
            *index += 4;
        }

        data
    }
}

impl FromBytes for (f32, f32, f32) {
    type Output = (f32, f32, f32);

    fn from_bytes(bytes: &[u8], index: &mut usize) -> Self::Output {
        let mut data = (0.0, 0.0, 0.0);

        let element_bytes = [
            bytes[*index],
            bytes[*index + 1],
            bytes[*index + 2],
            bytes[*index + 3],
        ];
        data.0 = f32::from_le_bytes(element_bytes);
        *index += 4;

        let element_bytes = [
            bytes[*index],
            bytes[*index + 1],
            bytes[*index + 2],
            bytes[*index + 3],
        ];
        data.1 = f32::from_le_bytes(element_bytes);
        *index += 4;

        let element_bytes = [
            bytes[*index],
            bytes[*index + 1],
            bytes[*index + 2],
            bytes[*index + 3],
        ];
        data.2 = f32::from_le_bytes(element_bytes);
        *index += 4;

        data
    }
}

impl<T: FromBytes> FromBytes for Vec<T> {
    type Output = Vec<T::Output>;

    fn from_bytes(bytes: &[u8], index: &mut usize) -> Self::Output {
        let mut data = vec![];

        let len_bytes = [
            bytes[*index],
            bytes[*index + 1],
            bytes[*index + 2],
            bytes[*index + 3],
            bytes[*index + 4],
            bytes[*index + 5],
            bytes[*index + 6],
            bytes[*index + 7],
        ];
        let len = u64::from_le_bytes(len_bytes);
        *index += 8;

        for i in 0..len {
            let element_data = T::from_bytes(bytes, index);
            data.push(element_data);
        }

        data
    }
}

impl FromBytes for NormalColliderData {
    type Output = NormalColliderData;

    fn from_bytes(bytes: &[u8], index: &mut usize) -> Self::Output {
        let indices = Vec::<VectorArray<u32>>::from_bytes(bytes, index);
        let vertices = Vec::<VectorTuple>::from_bytes(bytes, index);

        NormalColliderData { indices, vertices }
    }
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
        const COLLIDER_DATA: &'static [u8] =
            include_bytes!(concat!(env!("OUT_DIR"), "/collider_data"));
    };
}
