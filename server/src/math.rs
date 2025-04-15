use spacetimedb::SpacetimeType;

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct DbVector2 {
    pub x: f32,
    pub y: f32,
}

#[derive(SpacetimeType, Default, Debug, Clone, Copy, PartialEq)]
pub struct DbVector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct DbVector4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct DbMatrix4 {
    pub x: DbVector4,
    pub y: DbVector4,
    pub z: DbVector4,
    pub w: DbVector4,
}

#[derive(Debug, Default, Clone)]
pub struct Rectangle {
    pub min: DbVector3,
    pub max: DbVector3,
}

impl DbVector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

impl From<[f32; 3]> for DbVector3 {
    fn from(value: [f32; 3]) -> Self {
        Self {
            x: value[0],
            y: value[1],
            z: value[2],
        }
    }
}

impl std::ops::Mul<f32> for DbVector3 {
    type Output = DbVector3;

    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl std::ops::MulAssign<f32> for DbVector3 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl From<[f32; 4]> for DbVector4 {
    fn from(value: [f32; 4]) -> Self {
        Self {
            x: value[0],
            y: value[1],
            z: value[2],
            w: value[3],
        }
    }
}

impl From<[[f32; 4]; 4]> for DbMatrix4 {
    fn from(value: [[f32; 4]; 4]) -> Self {
        Self {
            x: value[0].into(),
            y: value[1].into(),
            z: value[2].into(),
            w: value[3].into(),
        }
    }
}

impl std::ops::Mul<DbVector4> for DbMatrix4 {
    type Output = DbVector4;

    fn mul(self, rhs: DbVector4) -> Self::Output {
        let x = self.x.x * rhs.x + self.y.x * rhs.x + self.z.x * rhs.x + self.w.x * rhs.x;
        let y = self.x.y * rhs.y + self.y.y * rhs.y + self.z.y * rhs.y + self.w.y * rhs.y;
        let z = self.x.z * rhs.z + self.y.z * rhs.z + self.z.z * rhs.z + self.w.z * rhs.z;
        let w = self.x.w * rhs.w + self.y.w * rhs.w + self.z.w * rhs.w + self.w.w * rhs.w;

        DbVector4 { x, y, z, w }
    }
}

impl From<DbVector3> for DbVector4 {
    fn from(value: DbVector3) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            w: 1.0,
        }
    }
}

impl From<DbVector4> for DbVector3 {
    fn from(value: DbVector4) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl Rectangle {
    // index: axis
    // 0: x; 1: y; 2: z
    pub fn min_point(&mut self, vector: [f32; 3]) {
        if vector[0] < self.min.x {
            self.min.x = vector[0];
        }
        if vector[1] < self.min.y {
            self.min.y = vector[1];
        }
        if vector[2] < self.min.z {
            self.min.z = vector[2];
        }
    }

    // index: axis
    // 0: x; 1: y; 2: z
    pub fn max_point(&mut self, vector: [f32; 3]) {
        if vector[0] > self.max.x {
            self.max.x = vector[0];
        }
        if vector[1] > self.max.y {
            self.max.y = vector[1];
        }
        if vector[2] > self.max.z {
            self.max.z = vector[2];
        }
    }

    pub fn has_inside(&self, vector: &DbVector3) -> bool {
        vector.x >= self.min.x
            && vector.x <= self.max.x
            && vector.y >= self.min.y
            && vector.y <= self.max.y
            && vector.z >= self.min.z
            && vector.z <= self.max.z
    }
}

impl std::ops::Mul<f32> for Rectangle {
    type Output = Rectangle;

    fn mul(self, rhs: f32) -> Self::Output {
        Rectangle {
            min: self.min * rhs,
            max: self.max * rhs,
        }
    }
}

impl std::ops::MulAssign<f32> for Rectangle {
    fn mul_assign(&mut self, rhs: f32) {
        self.min *= rhs;
        self.max *= rhs;
    }
}

impl std::ops::Add<&DbVector2> for DbVector2 {
    type Output = DbVector2;

    fn add(self, other: &DbVector2) -> DbVector2 {
        DbVector2 {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl std::ops::Add<DbVector2> for DbVector2 {
    type Output = DbVector2;

    fn add(self, other: DbVector2) -> DbVector2 {
        DbVector2 {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl std::ops::AddAssign<DbVector2> for DbVector2 {
    fn add_assign(&mut self, rhs: DbVector2) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl std::iter::Sum<DbVector2> for DbVector2 {
    fn sum<I: Iterator<Item = DbVector2>>(iter: I) -> Self {
        let mut r = DbVector2::new(0.0, 0.0);
        for val in iter {
            r += val;
        }
        r
    }
}

impl std::ops::Sub<&DbVector2> for DbVector2 {
    type Output = DbVector2;

    fn sub(self, other: &DbVector2) -> DbVector2 {
        DbVector2 {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl std::ops::Sub<DbVector2> for DbVector2 {
    type Output = DbVector2;

    fn sub(self, other: DbVector2) -> DbVector2 {
        DbVector2 {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl std::ops::SubAssign<DbVector2> for DbVector2 {
    fn sub_assign(&mut self, rhs: DbVector2) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl std::ops::Mul<f32> for DbVector2 {
    type Output = DbVector2;

    fn mul(self, other: f32) -> DbVector2 {
        DbVector2 {
            x: self.x * other,
            y: self.y * other,
        }
    }
}

impl std::ops::Div<f32> for DbVector2 {
    type Output = DbVector2;

    fn div(self, other: f32) -> DbVector2 {
        if other != 0.0 {
            DbVector2 {
                x: self.x / other,
                y: self.y / other,
            }
        } else {
            DbVector2 { x: 0.0, y: 0.0 }
        }
    }
}

impl DbVector2 {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn sqr_magnitude(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn normalized(self) -> DbVector2 {
        self / self.magnitude()
    }
}
