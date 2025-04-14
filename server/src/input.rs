use spacetimedb::SpacetimeType;

// 3 units per second
pub const MOVEMENT_SPEED: f32 = 3.0;

// 20 ms
pub const MOVEMENT_RATE_UPDATE: i64 = 20_000;

#[derive(SpacetimeType, Debug, Clone, Copy)]
pub enum InputKind {
    Forward,
    Backward,
    Left,
    Right,
}

#[derive(SpacetimeType, Default, Debug, Clone, Copy)]
pub struct DbInputState {
    pub forward: bool,
    pub backward: bool,
    pub left: bool,
    pub right: bool,
}
