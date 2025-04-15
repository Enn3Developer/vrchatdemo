pub mod input;
pub mod math;
mod rapier;

use crate::input::{DbInputState, InputKind};
use crate::math::{DbMatrix4, DbVector3, Rectangle};
use crate::rapier::{
    BROAD_PHASE, CCD_SOLVER, COLLIDER_SET, GRAVITY, IMPULSE_JOINT_SET, INTEGRATION_PARAMETERS,
    ISLAND_MANAGER, MULTIBODY_JOINT_SET, NARROW_PHASE, PHYSICS_PIPELINE, PLAYER_SHAPE,
    QUERY_PIPELINE, RIGID_BODY_SET,
};
use gltf::buffer::Data;
use gltf::mesh::util::ReadIndices;
use gltf::{buffer, mesh, Gltf, Node, Semantic};
use math::DbVector2;
use nalgebra::{Affine3, ArrayStorage, Matrix4, Quaternion, Transform, Transform3, UnitQuaternion};
use once_cell::sync::Lazy;
use rapier3d::control::{CharacterLength, KinematicCharacterController};
use rapier3d::parry::transformation::utils::transform;
use rapier3d::prelude::{
    ActiveCollisionTypes, ActiveEvents, AngVector, ColliderBuilder, Isometry, Point, QueryFilter,
    TriMeshFlags, Vector,
};
use spacetimedb::sys::raw::identity;
use spacetimedb::{ReducerContext, ScheduleAt, Table, TimeDuration, Timestamp};
use std::ops::{Deref, DerefMut};
// static BOUNDING_BOXES: Lazy<Vec<Rectangle>> = Lazy::new(|| {
//     let mut boxes = vec![];
//
//     let model_bytes = include_bytes!("../../client/src/assets/models/low_poly_stadium/scene.gltf");
//
//     // import model
//     let mut gltf = Gltf::from_slice(model_bytes.as_slice()).expect("can't load gltf model");
//     // let buffers = import_buffers(
//     //     &gltf.document,
//     //     Some(Path::new(
//     //         "../../client/src/assets/models/low_poly_stadium/scene.gltf",
//     //     )),
//     //     gltf.blob,
//     // )
//     // .expect("can't load buffer data for gltf model");
//
//     let mut buffers: Vec<Data> = vec![];
//     let gltf_buffers = gltf.document.buffers().clone();
//
//     for buffer in gltf_buffers {
//         if let gltf::buffer::Source::Bin = buffer.source() {
//             let blob = gltf.blob.take();
//
//             if let Some(mut blob) = blob {
//                 while blob.len() % 4 != 0 {
//                     blob.push(0);
//                 }
//
//                 buffers.push(buffer::Data(blob));
//             }
//         } else {
//             // here we assume the uri of the file is `scene.bin`
//             log::info!("buffer is from URI");
//
//             let mut blob =
//                 include_bytes!("../../client/src/assets/models/low_poly_stadium/scene.bin")
//                     .to_vec();
//
//             while blob.len() % 4 != 0 {
//                 blob.push(0);
//             }
//
//             buffers.push(buffer::Data(blob));
//         }
//     }
//
//     // let (gltf, buffers, _) =
//     //     gltf::import_slice(model_bytes.as_slice()).expect("can't import model");
//
//     for scene in gltf.document.scenes() {
//         for node in scene.nodes() {
//             get_bounding_boxes(&mut boxes, node);
//         }
//     }
//
//     // for mesh in gltf.document.meshes() {
//     //     let mut r#box = Rectangle::default();
//     //
//     //     for primitive in mesh.primitives() {
//     //         r#box.min = primitive.bounding_box().min.into();
//     //         r#box.max = primitive.bounding_box().max.into();
//     //
//     //         r#box *= 4.0;
//     //
//     //         // let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));
//     //         // if let Some(iter) = reader.read_positions() {
//     //         //     for mut vertex_position in iter {
//     //         //         // apply transformations to the vertex
//     //         //         transform_vertices(&mut vertex_position);
//     //         //         r#box.min_point(vertex_position);
//     //         //         r#box.max_point(vertex_position);
//     //         //     }
//     //         // }
//     //     }
//     //
//     //     boxes.push(r#box);
//     // }
//
//     boxes
// });
//
// fn get_bounding_boxes(boxes: &mut Vec<Rectangle>, node: gltf::Node) {
//     let mut children_len = 0;
//
//     for children in node.children() {
//         children_len += 1;
//         get_bounding_boxes(boxes, children);
//     }
//
//     if let Some(mesh) = node.mesh() {
//         let mut transform = node.transform().decomposed();
//         transform.2[0] *= 4.0;
//         transform.2[1] *= 4.0;
//         transform.2[2] *= 4.0;
//         let matrix: DbMatrix4 = gltf::scene::Transform::Decomposed {
//             translation: transform.0,
//             rotation: transform.1,
//             scale: transform.2,
//         }
//         .matrix()
//         .into();
//
//         for primitive in mesh.primitives() {
//             let mut r#box = Rectangle::default();
//
//             let min: DbVector3 = primitive.bounding_box().min.into();
//             let max: DbVector3 = primitive.bounding_box().max.into();
//
//             r#box.min = (matrix * min.into()).into();
//             r#box.max = (matrix * max.into()).into();
//
//             boxes.push(r#box);
//         }
//     }
// }

// Helper function to generate a random hex color using ReducerContext
fn generate_random_hex_color(ctx: &ReducerContext) -> String {
    // Generate random RGB values (keeping them slightly brighter by using 32-255 range)
    let r: u8 = ctx.random::<u8>() % 224 + 32; // 32-255 range
    let g: u8 = ctx.random::<u8>() % 224 + 32;
    let b: u8 = ctx.random::<u8>() % 224 + 32;

    // Format as hex color string
    format!("#{:02X}{:02X}{:02X}", r, g, b)
}

fn generate_collider_for_node(node: Node, transform: Transform3<f32>, buffers: &Vec<Data>) {
    let node_transform: Transform3<f32> = Transform3::from_matrix_unchecked(
        Matrix4::<f32>::from_data(ArrayStorage(node.transform().matrix())),
    );

    let transform: Transform3<f32> = node_transform * transform;

    for child in node.children() {
        generate_collider_for_node(child, transform, buffers);
    }

    if let Some(mesh) = node.mesh() {
        let mut vertices = vec![];
        let mut indices = vec![];

        for primitive in mesh.primitives() {
            let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));
            if let Some(positions) = reader.read_positions() {
                for vertex_position in positions {
                    let vertex = transform
                        * Point::new(vertex_position[0], vertex_position[1], vertex_position[2]);

                    vertices.push(vertex);
                }
            }

            if let Some(primitive_indices) = reader.read_indices() {
                let mut vector: [u32; 3] = [0, 0, 0];
                let mut i = 0;

                for index in primitive_indices.into_u32() {
                    vector[i] = index;
                    i += 1;

                    if i >= 3 {
                        indices.push(vector);
                        i = 0;
                    }
                }
            }
        }

        let transform = node.transform().decomposed();
        let rotation = UnitQuaternion::from_quaternion(Quaternion::from(transform.1));
        let rotation_euler = rotation.euler_angles();

        log::info!(
            "Mesh #{} -- vertices: {} -- indices: {} -- position: {:?}",
            mesh.index(),
            vertices.len(),
            indices.len(),
            transform.0
        );

        // create collider with mesh data
        let mut collider = ColliderBuilder::trimesh_with_flags(
            vertices,
            indices,
            TriMeshFlags::FIX_INTERNAL_EDGES,
        )
        .expect("can't create collider from gltf model")
        // .position(transform.0.into())
        .rotation(AngVector::new(
            rotation_euler.0,
            rotation_euler.1,
            rotation_euler.2,
        ))
        .build();

        // enable collision with players
        collider.set_active_collision_types(
            ActiveCollisionTypes::default() | ActiveCollisionTypes::KINEMATIC_FIXED,
        );

        // add collider to the world
        COLLIDER_SET
            .lock()
            .expect("can't lock COLLIDER_SET; COLLIDER_SET should be free when `init` is running")
            .insert(collider);
    }
}

#[spacetimedb::table(name = player, public)]
#[spacetimedb::table(name = logged_out_player, public)]
#[derive(Clone, Debug)]
pub struct Player {
    #[primary_key]
    identity: spacetimedb::Identity,

    #[unique]
    #[auto_inc]
    player_id: u32,

    username: Option<String>,

    // Store the player's color as a hex string (e.g. "#FF00FF")
    // If not specified, this will be automatically generated on client side
    pub hex_color: Option<String>,

    pub position: DbVector2,
    pub rotation_yaw: f32,
    pub animation_state: Option<String>,

    pub input_state: DbInputState,
}

#[spacetimedb::table(name = update_player_schedule, scheduled(update_player_scheduled))]
struct UpdatePlayerSchedule {
    #[primary_key]
    #[auto_inc]
    scheduled_id: u64,

    scheduled_at: ScheduleAt,

    last_update: Timestamp,
}

#[spacetimedb::reducer(init)]
pub fn init(ctx: &ReducerContext) {
    let model_bytes = include_bytes!("../../client/src/assets/models/low_poly_stadium/scene.gltf");

    // import model
    let mut gltf = Gltf::from_slice(model_bytes.as_slice()).expect("can't load gltf model");

    let mut buffers: Vec<Data> = vec![];
    let gltf_buffers = gltf.document.buffers().clone();

    for buffer in gltf_buffers {
        if let gltf::buffer::Source::Bin = buffer.source() {
            let blob = gltf.blob.take();

            if let Some(mut blob) = blob {
                while blob.len() % 4 != 0 {
                    blob.push(0);
                }

                buffers.push(buffer::Data(blob));
            }
        } else {
            // here we assume the uri of the file is `scene.bin`
            log::info!("buffer is from URI");

            let mut blob =
                include_bytes!("../../client/src/assets/models/low_poly_stadium/scene.bin")
                    .to_vec();

            while blob.len() % 4 != 0 {
                blob.push(0);
            }

            buffers.push(buffer::Data(blob));
        }
    }

    for scene in gltf.scenes() {
        for node in scene.nodes() {
            let initial_scaling = 1.0;
            let identity: Transform3<f32> = Transform3::from_matrix_unchecked(
                Matrix4::<f32>::identity().scale(initial_scaling),
            );

            generate_collider_for_node(node, identity, &buffers);
        }
    }

    let schedule_timestamp = TimeDuration::from_micros(input::MOVEMENT_RATE_UPDATE);

    ctx.db
        .update_player_schedule()
        .insert(UpdatePlayerSchedule {
            scheduled_id: 0,
            scheduled_at: ScheduleAt::Interval(schedule_timestamp),
            last_update: ctx.timestamp,
        });
}

#[spacetimedb::reducer(client_connected)]
pub fn connect(ctx: &ReducerContext) -> Result<(), String> {
    if let Some(player) = ctx.db.logged_out_player().identity().find(&ctx.sender) {
        // Make sure the player's color is preserved when reconnecting
        log::info!("Player reconnected with color: {:?}", player.hex_color);

        // If the player doesn't have a color, generate one now
        let player = if player.hex_color.is_none() {
            let color = generate_random_hex_color(ctx);
            log::info!("Assigning new color to reconnecting player: {}", color);

            let mut updated_player = player.clone();
            updated_player.hex_color = Some(color);
            updated_player
        } else {
            player.clone()
        };

        ctx.db.player().insert(player.clone());
        ctx.db
            .logged_out_player()
            .identity()
            .delete(&player.identity);
    } else {
        // Generate a random hex color for the new player
        let color = generate_random_hex_color(ctx);
        log::info!("Generated new color for new player: {}", color);

        ctx.db.player().try_insert(Player {
            identity: ctx.sender,
            player_id: 0,
            username: None,
            hex_color: Some(color),
            position: DbVector2::new(0.0, 0.0),
            rotation_yaw: 0.0,
            animation_state: None,
            input_state: DbInputState::default(),
        })?;
    };

    Ok(())
}

#[spacetimedb::reducer(client_disconnected)]
pub fn disconnect(ctx: &ReducerContext) -> Result<(), String> {
    let mut player = ctx
        .db
        .player()
        .identity()
        .find(&ctx.sender)
        .ok_or("Player not found")?;

    // reset input state for disconnecting players
    player.input_state = DbInputState::default();

    //let player_id = player.player_id;
    ctx.db.logged_out_player().insert(player);
    ctx.db.player().identity().delete(&ctx.sender);

    Ok(())
}

#[spacetimedb::reducer]
pub fn update_player_rotation(ctx: &ReducerContext, rotation: f32) {
    if let Some(mut player) = ctx.db.player().identity().find(&ctx.sender) {
        player.rotation_yaw = rotation;
        ctx.db.player().identity().update(player);
        log::info!("Updated player rotation");
    } else {
        log::error!("Player not found");
    }
}

#[spacetimedb::reducer]
pub fn update_player_animation_state(ctx: &ReducerContext, animation_state: String) {
    if let Some(mut player) = ctx.db.player().identity().find(&ctx.sender) {
        // Log the value *before* the move
        log::info!("Updating player animation state to: {}", animation_state);
        player.animation_state = Some(animation_state); // Move happens here
        ctx.db.player().identity().update(player); // Update the player in the database
    } else {
        log::error!("Player not found for animation update");
    }
}

#[spacetimedb::reducer]
pub fn update_player_movement(ctx: &ReducerContext, kind: InputKind, enabled: bool) {
    if let Some(mut player) = ctx.db.player().identity().find(&ctx.sender) {
        log::info!("Updating movement {kind:?}");

        // Modify the player input state
        match kind {
            InputKind::Forward => player.input_state.forward = enabled,
            InputKind::Backward => player.input_state.backward = enabled,
            InputKind::Left => player.input_state.left = enabled,
            InputKind::Right => player.input_state.right = enabled,
        }

        // Update the player in the database
        ctx.db.player().identity().update(player);
    } else {
        log::error!("Player not found for movement event");
    }
}

#[spacetimedb::reducer]
fn update_player_scheduled(ctx: &ReducerContext, args: UpdatePlayerSchedule) {
    // Only the server can call this reducer
    if ctx.sender != ctx.identity() {
        log::error!("User not authorized to call this reducer");
        return;
    }

    if args.last_update >= ctx.timestamp {
        log::error!("Last update happened *after* this scheduled update");
    }

    // Compute delta since last update
    let delta = input::MOVEMENT_RATE_UPDATE as f32 / 1_000_000.0;

    // let delta = ctx
    //     .timestamp
    //     .duration_since(args.last_update)
    //     .expect("Last update is greater than current timestamp or the difference overflows i64")
    //     .as_secs_f32();

    let speed = input::MOVEMENT_SPEED * delta;

    let mut character_controller = KinematicCharacterController::default();
    character_controller.up = Vector::y_axis();
    character_controller.offset = CharacterLength::Absolute(0.01);
    // Don’t allow climbing slopes larger than 45 degrees
    character_controller.max_slope_climb_angle = 45_f32.to_radians();
    // Automatically slide down on slopes smaller than 30 degrees
    character_controller.min_slope_slide_angle = 30_f32.to_radians();
    // Snap to the ground if the vertical distance to the ground is smaller than 0.2
    character_controller.snap_to_ground = Some(CharacterLength::Absolute(0.2));

    for mut player in ctx.db.player().iter() {
        // compute cosine and sine of the player's rotation
        let cos = (-player.rotation_yaw).cos(); // negate because the yaw is clockwise, but we need an anticlockwise yaw
        let sin = (-player.rotation_yaw).sin();

        // convert input state to input vector
        let (x, y) = player.input_state.into();

        // rotate input vector
        let nx = x * cos - y * sin;
        let ny = x * sin + y * cos;

        // normalize rotated input vector and multiply it by speed * delta
        let mut movement = DbVector2::new(nx, ny).normalized() * speed;

        // apply movement only if the player is moving
        if movement != DbVector2::new(0.0, 0.0) {
            let pos = Isometry::translation(player.position.x, 1.0, player.position.y);
            let translation = Vector::new(movement.x, 0.0, movement.y);
            let new_movement = &mut movement;

            log::info!("colliders: {}", COLLIDER_SET.lock().unwrap().len());

            let effective_movement = character_controller.move_shape(
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
                &pos,
                translation,
                QueryFilter::exclude_dynamic(),
                |collision| {
                    log::info!(
                        "Collision detected at {}; movement: {new_movement:?}; applied: {}",
                        collision.character_pos,
                        collision.translation_applied
                    );
                },
            );

            log::info!("movement before: {movement:?}");

            movement.x = effective_movement.translation.x;
            movement.y = effective_movement.translation.z;

            log::info!("movement after: {movement:?}");
            player.position += movement;

            // update the player
            ctx.db.player().identity().update(player);
        }

        // // schedule new update
        // let schedule_timestamp =
        //     ctx.timestamp + TimeDuration::from_micros(input::MOVEMENT_RATE_UPDATE);
        // ctx.db
        //     .update_player_schedule()
        //     .insert(UpdatePlayerSchedule {
        //         scheduled_id: 0,
        //         scheduled_at: schedule_timestamp.into(),
        //         last_update: ctx.timestamp,
        //     });
    }

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
