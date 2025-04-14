pub mod input;
pub mod math;

use crate::input::{DbInputState, InputKind};
use math::DbVector2;
use spacetimedb::{ReducerContext, ScheduleAt, Table, TimeDuration, Timestamp};

// Helper function to generate a random hex color using ReducerContext
fn generate_random_hex_color(ctx: &ReducerContext) -> String {
    // Generate random RGB values (keeping them slightly brighter by using 32-255 range)
    let r: u8 = ctx.random::<u8>() % 224 + 32; // 32-255 range
    let g: u8 = ctx.random::<u8>() % 224 + 32;
    let b: u8 = ctx.random::<u8>() % 224 + 32;

    // Format as hex color string
    format!("#{:02X}{:02X}{:02X}", r, g, b)
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

#[spacetimedb::table(name = update_player_schedule)]
struct UpdatePlayerSchedule {
    #[primary_key]
    #[auto_inc]
    scheduled_id: u64,

    scheduled_at: ScheduleAt,

    // Player to update
    player_id: u32,
    last_update: Timestamp,
}

#[spacetimedb::reducer(client_connected)]
pub fn connect(ctx: &ReducerContext) -> Result<(), String> {
    let player_id = if let Some(player) = ctx.db.logged_out_player().identity().find(&ctx.sender) {
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

        player.player_id
    } else {
        // Generate a random hex color for the new player
        let color = generate_random_hex_color(ctx);
        log::info!("Generated new color for new player: {}", color);

        let player = ctx.db.player().try_insert(Player {
            identity: ctx.sender,
            player_id: 0,
            username: None,
            hex_color: Some(color),
            position: DbVector2::new(0.0, 0.0),
            rotation_yaw: 0.0,
            animation_state: None,
            input_state: DbInputState::default(),
        })?;

        player.player_id
    };

    let schedule_timestamp = ctx.timestamp + TimeDuration::from_micros(input::MOVEMENT_RATE_UPDATE);

    ctx.db
        .update_player_schedule()
        .insert(UpdatePlayerSchedule {
            scheduled_id: 0,
            scheduled_at: schedule_timestamp.into(),
            player_id,
            last_update: ctx.timestamp,
        });

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
    let delta = ctx
        .timestamp
        .duration_since(args.last_update)
        .expect("Last update is greater than current timestamp or the difference overflows i64")
        .as_secs_f32();

    let speed = input::MOVEMENT_SPEED * delta;

    if let Some(mut player) = ctx.db.player().player_id().find(args.player_id) {
        // TODO: apply movement

        let schedule_timestamp =
            ctx.timestamp + TimeDuration::from_micros(input::MOVEMENT_RATE_UPDATE);
        ctx.db
            .update_player_schedule()
            .insert(UpdatePlayerSchedule {
                scheduled_id: 0,
                scheduled_at: schedule_timestamp.into(),
                player_id: args.player_id,
                last_update: ctx.timestamp,
            });
    } else {
        log::error!("Player not found for scheduled movement update");
    }
}
