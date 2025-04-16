pub mod input;
pub mod math;
pub mod rapier;

use crate::input::{DbInputState, InputKind};
use crate::rapier::{
    default_character_controller, move_shape, physics_step, FromBytes, NormalColliderData,
    COLLIDER_SET,
};
use math::DbVector2;
use rapier3d::prelude::{
    ActiveCollisionTypes, ColliderBuilder, Isometry, Point, TriMeshFlags, Vector,
};
use spacetimedb::{ReducerContext, ScheduleAt, Table, TimeDuration};

import_collider_data!();

// Helper function to generate a random hex color using ReducerContext
fn generate_random_hex_color(ctx: &ReducerContext) -> String {
    // Generate random RGB values (keeping them slightly brighter by using 32-255 range)
    let r: u8 = ctx.random::<u8>() % 224 + 32; // 32-255 range
    let g: u8 = ctx.random::<u8>() % 224 + 32;
    let b: u8 = ctx.random::<u8>() % 224 + 32;

    // Format as hex color string
    format!("#{:02X}{:02X}{:02X}", r, g, b)
}

fn generate_colliders() {
    let mut index = 0;
    let colliders_data = Vec::<NormalColliderData>::from_bytes(COLLIDER_DATA, &mut index);

    log::info!("Found {} colliders", colliders_data.len());

    for collider_data in colliders_data.iter() {
        let vertices = collider_data
            .vertices
            .iter()
            .map(|vertex| Point::<f32>::new(vertex.0, vertex.1, vertex.2))
            .collect();

        // create collider with mesh data
        let mut collider = ColliderBuilder::trimesh_with_flags(
            vertices,
            collider_data.indices.clone(),
            TriMeshFlags::FIX_INTERNAL_EDGES,
        )
        .expect("can't create collider from gltf model")
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
}

#[spacetimedb::reducer(init)]
pub fn init(ctx: &ReducerContext) {
    generate_colliders();

    let schedule_timestamp = TimeDuration::from_micros(input::MOVEMENT_RATE_UPDATE);

    ctx.db
        .update_player_schedule()
        .insert(UpdatePlayerSchedule {
            scheduled_id: 0,
            scheduled_at: ScheduleAt::Interval(schedule_timestamp),
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
fn update_player_scheduled(ctx: &ReducerContext, _args: UpdatePlayerSchedule) {
    // Only the server can call this reducer
    if ctx.sender != ctx.identity() {
        log::error!("User not authorized to call this reducer");
        return;
    }

    // Compute delta since last update
    // Alternative for delta: save last timestamp somewhere and check against current timestamp
    let delta = input::MOVEMENT_RATE_UPDATE as f32 / 1_000_000.0;

    let speed = input::MOVEMENT_SPEED * delta;

    // Setup character controller
    let character_controller = default_character_controller();

    // for every player online
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
            // setup data
            let pos = Isometry::translation(player.position.x, 1.0, player.position.y);
            let translation = Vector::new(movement.x, 0.0, movement.y);

            // compute effective movement of the player
            let effective_movement = move_shape(character_controller, delta, &pos, translation);

            // apply effective movement
            movement.x = effective_movement.translation.x;
            movement.y = effective_movement.translation.z;

            // add movement to player position
            player.position += movement;

            // update the player
            ctx.db.player().identity().update(player);
        }
    }

    // compute the next physics step
    physics_step();
}
