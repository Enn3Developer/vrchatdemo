use spacetimedb::SpacetimeType;

// 5 units per second
pub const MOVEMENT_SPEED: f32 = 5.0;

// 30 ms
pub const MOVEMENT_RATE_UPDATE: i64 = 30_000;

pub trait ToF32 {
    fn to_f32(self) -> f32;
}

impl ToF32 for bool {
    fn to_f32(self) -> f32 {
        if self {
            1.0
        } else {
            0.0
        }
    }
}

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

impl From<DbInputState> for (f32, f32) {
    fn from(value: DbInputState) -> Self {
        let x = value.left.to_f32() - value.right.to_f32();
        let y = value.forward.to_f32() - value.backward.to_f32();

        (x, y)
    }
}
