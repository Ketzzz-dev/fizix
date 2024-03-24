use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};
use log::warn;

// Structures
#[derive(Debug, Copy, Clone)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64
}

#[derive(Debug, Copy, Clone)]
pub struct Matrix3x3 {
    pub m11: f64, pub m12: f64, pub m13: f64,
    pub m21: f64, pub m22: f64, pub m23: f64,
    pub m31: f64, pub m32: f64, pub m33: f64
}

#[derive(Debug, Copy, Clone)]
pub struct Matrix4x4 {
    pub m11: f64, pub m12: f64, pub m13: f64, pub m14: f64,
    pub m21: f64, pub m22: f64, pub m23: f64, pub m24: f64,
    pub m31: f64, pub m32: f64, pub m33: f64, pub m34: f64,
    pub m41: f64, pub m42: f64, pub m43: f64, pub m44: f64
}

#[derive(Debug, Copy, Clone)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64
}

// Struct Implementations
impl Vector3 {
    pub const ZERO: Vector3 = Vector3 { x: 0.0, y: 0.0, z: 0.0 };
    pub const X_AXIS: Vector3 = Vector3 { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y_AXIS: Vector3 = Vector3 { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z_AXIS: Vector3 = Vector3 { x: 0.0, y: 0.0, z: 1.0 };

    pub fn dot(&self, other: &Vector3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    pub fn cross(&self, other: &Vector3) -> Vector3 {
        Vector3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x
        }
    }

    pub fn magnitude_sq(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
    pub fn magnitude(&self) -> f64 {
        self.magnitude_sq().sqrt()
    }

    pub fn normalized(&self) -> Vector3 {
        let magnitude = self.magnitude_sq();

        if magnitude == 0.0 {
            warn!("Zero magnitude!");

            return *self;
        }

        *self * (1.0 / magnitude.sqrt())
    }
}

impl Neg for Vector3 {
    type Output = Vector3;

    fn neg(self) -> Self::Output {
        Vector3 {
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }
}

impl Mul<f64> for Vector3 {
    type Output = Vector3;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector3 {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs
        }
    }
}

// backwards compatibility
impl Mul<Vector3> for f64 {
    type Output = Vector3;

    fn mul(self, rhs: Vector3) -> Self::Output {
        rhs * self
    }
}

impl MulAssign<f64> for Vector3 {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Add for Vector3 {
    type Output = Vector3;

    fn add(self, rhs: Self) -> Self::Output {
        Vector3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z
        }
    }
}
impl AddAssign for Vector3 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vector3 {
    type Output = Vector3;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z
        }
    }
}
impl SubAssign for Vector3 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Matrix3x3 {
    pub const IDENTITY: Matrix3x3 = Matrix3x3 {
        m11: 1.0, m12: 0.0, m13: 0.0,
        m21: 0.0, m22: 1.0, m23: 0.0,
        m31: 0.0, m32: 0.0, m33: 1.0
    };

    pub fn determinant(&self) -> f64 {
        self.m11 * self.m22 * self.m33
            - self.m11 * self.m23 * self.m32
            - self.m33 * self.m12 * self.m21
            + self.m23 * self.m12 * self.m31
            + self.m32 * self.m21 * self.m13
            - self.m22 * self.m31 * self.m13
    }

    pub fn inverse(&self) -> Matrix3x3 {
        let determinant = self.determinant();

        if determinant == 0.0 {
            warn!("Zero determinant!");

            return *self;
        }

        let determinant = 1.0 / determinant;

        Matrix3x3 {
            m11: (self.m22 * self.m33 - self.m23 * self.m32) * determinant,
            m12: (self.m13 * self.m32 - self.m12 * self.m33) * determinant,
            m13: (self.m12 * self.m23 - self.m13 * self.m22) * determinant,

            m21: (self.m23 * self.m31 - self.m21 * self.m33) * determinant,
            m22: (self.m11 * self.m33 - self.m13 * self.m31) * determinant,
            m23: (self.m13 * self.m21 - self.m11 * self.m23) * determinant,

            m31: (self.m21 * self.m32 - self.m22 * self.m31) * determinant,
            m32: (self.m12 * self.m31 - self.m11 * self.m32) * determinant,
            m33: (self.m11 * self.m22 - self.m12 * self.m21) * determinant
        }
    }

    pub fn transpose(&self) -> Matrix3x3 {
        Matrix3x3 {
            m11: self.m11, m12: self.m21, m13: self.m31,
            m21: self.m12, m22: self.m22, m23: self.m32,
            m31: self.m13, m32: self.m23, m33: self.m33
        }
    }
}

impl Mul<Vector3> for Matrix3x3 {
    type Output = Vector3;

    fn mul(self, rhs: Vector3) -> Self::Output {
        Vector3 {
            x: self.m11 * rhs.x + self.m12 * rhs.y + self.m13 * rhs.z,
            y: self.m21 * rhs.x + self.m22 * rhs.y + self.m23 * rhs.z,
            z: self.m31 * rhs.x + self.m32 * rhs.y + self.m33 * rhs.z
        }
    }
}

impl Mul for Matrix3x3 {
    type Output = Matrix3x3;

    fn mul(self, rhs: Matrix3x3) -> Self::Output {
        Matrix3x3 {
            m11: self.m11 * rhs.m11 + self.m12 * rhs.m21 + self.m13 * rhs.m31,
            m12: self.m11 * rhs.m12 + self.m12 * rhs.m22 + self.m13 * rhs.m32,
            m13: self.m11 * rhs.m13 + self.m12 * rhs.m23 + self.m13 * rhs.m33,

            m21: self.m21 * rhs.m11 + self.m22 * rhs.m21 + self.m23 * rhs.m31,
            m22: self.m21 * rhs.m12 + self.m22 * rhs.m22 + self.m23 * rhs.m32,
            m23: self.m21 * rhs.m13 + self.m22 * rhs.m23 + self.m23 * rhs.m33,

            m31: self.m31 * rhs.m11 + self.m32 * rhs.m21 + self.m33 * rhs.m31,
            m32: self.m31 * rhs.m12 + self.m32 * rhs.m22 + self.m33 * rhs.m32,
            m33: self.m31 * rhs.m13 + self.m32 * rhs.m23 + self.m33 * rhs.m33
        }
    }
}
impl MulAssign<Matrix3x3> for Matrix3x3 {
    fn mul_assign(&mut self, rhs: Matrix3x3) {
        let m11 = self.m11 * rhs.m11 + self.m12 * rhs.m21 + self.m13 * rhs.m31;
        let m12 = self.m11 * rhs.m12 + self.m12 * rhs.m22 + self.m13 * rhs.m32;
        let m13 = self.m11 * rhs.m13 + self.m12 * rhs.m23 + self.m13 * rhs.m33;

        let m21 = self.m21 * rhs.m11 + self.m22 * rhs.m21 + self.m23 * rhs.m31;
        let m22 = self.m21 * rhs.m12 + self.m22 * rhs.m22 + self.m23 * rhs.m32;
        let m23 = self.m21 * rhs.m13 + self.m22 * rhs.m23 + self.m23 * rhs.m33;

        let m31 = self.m31 * rhs.m11 + self.m32 * rhs.m21 + self.m33 * rhs.m31;
        let m32 = self.m31 * rhs.m12 + self.m32 * rhs.m22 + self.m33 * rhs.m32;
        let m33 = self.m31 * rhs.m13 + self.m32 * rhs.m23 + self.m33 * rhs.m33;

        self.m11 = m11;
        self.m12 = m12;
        self.m13 = m13;

        self.m21 = m21;
        self.m22 = m22;
        self.m23 = m23;

        self.m31 = m31;
        self.m32 = m32;
        self.m33 = m33;
    }
}

impl Matrix4x4 {
    pub const IDENTITY: Matrix4x4 = Matrix4x4 {
        m11: 1.0, m12: 0.0, m13: 0.0, m14: 0.0,
        m21: 0.0, m22: 1.0, m23: 0.0, m24: 0.0,
        m31: 0.0, m32: 0.0, m33: 1.0, m34: 0.0,
        m41: 0.0, m42: 0.0, m43: 0.0, m44: 1.0
    };

    pub fn determinant(&self) -> f64 {
        self.m11 * self.m22 * self.m33 * self.m44 + self.m11 * self.m23 * self.m34 * self.m42 + self.m11 * self.m24 * self.m32 * self.m43
            - self.m11 * self.m24 * self.m33 * self.m42 - self.m11 * self.m23 * self.m32 * self.m44 - self.m11 * self.m22 * self.m34 * self.m43
            - self.m12 * self.m21 * self.m33 * self.m44 - self.m13 * self.m21 * self.m34 * self.m42 - self.m14 * self.m21 * self.m32 * self.m43
            + self.m14 * self.m21 * self.m33 * self.m42 + self.m13 * self.m21 * self.m32 * self.m44 + self.m12 * self.m21 * self.m34 * self.m43
            + self.m12 * self.m23 * self.m31 * self.m44 + self.m13 * self.m24 * self.m31 * self.m42 + self.m14 * self.m22 * self.m31 * self.m43
            - self.m14 * self.m23 * self.m31 * self.m42 - self.m13 * self.m22 * self.m31 * self.m44 - self.m12 * self.m24 * self.m31 * self.m43
            - self.m12 * self.m23 * self.m34 * self.m41 - self.m13 * self.m24 * self.m32 * self.m41 - self.m14 * self.m22 * self.m33 * self.m41
            + self.m14 * self.m23 * self.m32 * self.m41 + self.m13 * self.m22 * self.m34 * self.m41 + self.m12 * self.m24 * self.m33 * self.m41
    }

    pub fn inverse(&self) -> Matrix4x4 {
        let determinant = self.determinant();

        if determinant == 0.0 {
            warn!("Zero determinant!");

            return *self;
        }

        let determinant = 1.0 / determinant;

        Matrix4x4 {
            m11: (self.m22 * self.m33 * self.m44 + self.m23 * self.m34 * self.m42 + self.m24 * self.m32 * self.m43
                - self.m24 * self.m33 * self.m42 - self.m23 * self.m32 * self.m44 - self.m22 * self.m34 * self.m43) * determinant,
            m12: (-self.m12 * self.m33 * self.m44 - self.m13 * self.m34 * self.m42 - self.m14 * self.m32 * self.m43
                + self.m14 * self.m33 * self.m42 + self.m13 * self.m32 * self.m44 + self.m12 * self.m34 * self.m43) * determinant,
            m13: (self.m12 * self.m23 * self.m44 + self.m13 * self.m24 * self.m42 + self.m14 * self.m22 * self.m43
                - self.m14 * self.m23 * self.m42 - self.m13 * self.m22 * self.m44 - self.m12 * self.m24 * self.m43) * determinant,
            m14: (-self.m12 * self.m23 * self.m34 - self.m13 * self.m24 * self.m32 - self.m14 * self.m22 * self.m33
                + self.m14 * self.m23 * self.m32 + self.m13 * self.m22 * self.m34 + self.m12 * self.m24 * self.m33) * determinant,

            m21: (-self.m21 * self.m33 * self.m44 - self.m23 * self.m34 * self.m41 - self.m24 * self.m31 * self.m43
                + self.m24 * self.m33 * self.m41 + self.m23 * self.m31 * self.m44 + self.m21 * self.m34 * self.m43) * determinant,
            m22: (self.m11 * self.m33 * self.m44 + self.m13 * self.m34 * self.m41 + self.m14 * self.m31 * self.m43
                - self.m14 * self.m33 * self.m41 - self.m13 * self.m31 * self.m44 - self.m11 * self.m34 * self.m43) * determinant,
            m23: (-self.m11 * self.m23 * self.m44 - self.m13 * self.m24 * self.m41 - self.m14 * self.m21 * self.m43
                + self.m14 * self.m23 * self.m41 + self.m13 * self.m21 * self.m44 + self.m11 * self.m24 * self.m43) * determinant,
            m24: (self.m11 * self.m23 * self.m34 + self.m13 * self.m24 * self.m31 + self.m14 * self.m21 * self.m33
                - self.m14 * self.m23 * self.m31 - self.m13 * self.m21 * self.m34 - self.m11 * self.m24 * self.m33) * determinant,

            m31: (self.m21 * self.m32 * self.m44 + self.m22 * self.m34 * self.m41 + self.m24 * self.m31 * self.m42
                - self.m24 * self.m32 * self.m41 - self.m22 * self.m31 * self.m44 - self.m21 * self.m34 * self.m42) * determinant,
            m32: (-self.m11 * self.m32 * self.m44 - self.m12 * self.m34 * self.m41 - self.m14 * self.m31 * self.m42
                + self.m14 * self.m32 * self.m41 + self.m12 * self.m31 * self.m44 + self.m11 * self.m34 * self.m42) * determinant,
            m33: (self.m11 * self.m22 * self.m44 + self.m12 * self.m24 * self.m41 + self.m14 * self.m21 * self.m42
                - self.m14 * self.m22 * self.m41 - self.m12 * self.m21 * self.m44 - self.m11 * self.m24 * self.m42) * determinant,
            m34: (-self.m11 * self.m22 * self.m34 - self.m12 * self.m24 * self.m31 - self.m14 * self.m21 * self.m32
                + self.m14 * self.m22 * self.m31 + self.m12 * self.m21 * self.m34 + self.m11 * self.m24 * self.m32) * determinant,

            m41: (-self.m21 * self.m32 * self.m43 - self.m22 * self.m33 * self.m41 - self.m23 * self.m31 * self.m42
                + self.m23 * self.m32 * self.m41 + self.m22 * self.m31 * self.m43 + self.m21 * self.m33 * self.m42) * determinant,
            m42: (self.m11 * self.m32 * self.m43 + self.m12 * self.m33 * self.m41 + self.m13 * self.m31 * self.m42
                - self.m13 * self.m32 * self.m41 - self.m12 * self.m31 * self.m43 - self.m11 * self.m33 * self.m42) * determinant,
            m43: (-self.m11 * self.m22 * self.m43 - self.m12 * self.m23 * self.m41 - self.m13 * self.m21 * self.m42
                + self.m13 * self.m22 * self.m41 + self.m12 * self.m21 * self.m43 + self.m11 * self.m23 * self.m42) * determinant,
            m44: (self.m11 * self.m22 * self.m33 + self.m12 * self.m23 * self.m31 + self.m13 * self.m21 * self.m32
                - self.m13 * self.m22 * self.m31 - self.m12 * self.m21 * self.m33 - self.m11 * self.m23 * self.m32) * determinant
        }
    }

    pub fn transpose(&self) -> Matrix4x4 {
        Matrix4x4 {
            m11: self.m11, m12: self.m21, m13: self.m31, m14: self.m41,
            m21: self.m12, m22: self.m22, m23: self.m32, m24: self.m42,
            m31: self.m13, m32: self.m23, m33: self.m33, m34: self.m43,
            m41: self.m14, m42: self.m24, m43: self.m34, m44: self.m44
        }
    }
}

impl Mul<Vector3> for Matrix4x4 {
    type Output = Vector3;

    fn mul(self, rhs: Vector3) -> Self::Output {
        Vector3 {
            x: self.m11 * rhs.x + self.m12 * rhs.y + self.m13 * rhs.z + self.m14,
            y: self.m21 * rhs.x + self.m22 * rhs.y + self.m23 * rhs.z + self.m24,
            z: self.m31 * rhs.x + self.m32 * rhs.y + self.m33 * rhs.z + self.m34
        }
    }
}

impl Mul for Matrix4x4 {
    type Output = Matrix4x4;

    fn mul(self, rhs: Matrix4x4) -> Self::Output {
        Matrix4x4 {
            m11: self.m11 * rhs.m11 + self.m12 * rhs.m21 + self.m13 * rhs.m31 + self.m14 * rhs.m41,
            m12: self.m11 * rhs.m12 + self.m12 * rhs.m22 + self.m13 * rhs.m32 + self.m14 * rhs.m42,
            m13: self.m11 * rhs.m13 + self.m12 * rhs.m23 + self.m13 * rhs.m33 + self.m14 * rhs.m43,
            m14: self.m11 * rhs.m14 + self.m12 * rhs.m24 + self.m13 * rhs.m34 + self.m14 * rhs.m44,

            m21: self.m21 * rhs.m11 + self.m22 * rhs.m21 + self.m23 * rhs.m31 + self.m24 * rhs.m41,
            m22: self.m21 * rhs.m12 + self.m22 * rhs.m22 + self.m23 * rhs.m32 + self.m24 * rhs.m42,
            m23: self.m21 * rhs.m13 + self.m22 * rhs.m23 + self.m23 * rhs.m33 + self.m24 * rhs.m43,
            m24: self.m21 * rhs.m14 + self.m22 * rhs.m24 + self.m23 * rhs.m34 + self.m24 * rhs.m44,

            m31: self.m31 * rhs.m11 + self.m32 * rhs.m21 + self.m33 * rhs.m31 + self.m34 * rhs.m41,
            m32: self.m31 * rhs.m12 + self.m32 * rhs.m22 + self.m33 * rhs.m32 + self.m34 * rhs.m42,
            m33: self.m31 * rhs.m13 + self.m32 * rhs.m23 + self.m33 * rhs.m33 + self.m34 * rhs.m43,
            m34: self.m31 * rhs.m14 + self.m32 * rhs.m24 + self.m33 * rhs.m34 + self.m34 * rhs.m44,

            m41: self.m41 * rhs.m11 + self.m42 * rhs.m21 + self.m43 * rhs.m31 + self.m44 * rhs.m41,
            m42: self.m41 * rhs.m12 + self.m42 * rhs.m22 + self.m43 * rhs.m32 + self.m44 * rhs.m42,
            m43: self.m41 * rhs.m13 + self.m42 * rhs.m23 + self.m43 * rhs.m33 + self.m44 * rhs.m43,
            m44: self.m41 * rhs.m14 + self.m42 * rhs.m24 + self.m43 * rhs.m34 + self.m44 * rhs.m44
        }
    }
}

impl MulAssign for Matrix4x4 {
    fn mul_assign(&mut self, rhs: Self) {
        let m11 = self.m11 * rhs.m11 + self.m12 * rhs.m21 + self.m13 * rhs.m31 + self.m14 * rhs.m41;
        let m12 = self.m11 * rhs.m12 + self.m12 * rhs.m22 + self.m13 * rhs.m32 + self.m14 * rhs.m42;
        let m13 = self.m11 * rhs.m13 + self.m12 * rhs.m23 + self.m13 * rhs.m33 + self.m14 * rhs.m43;
        let m14 = self.m11 * rhs.m14 + self.m12 * rhs.m24 + self.m13 * rhs.m34 + self.m14 * rhs.m44;

        let m21 = self.m21 * rhs.m11 + self.m22 * rhs.m21 + self.m23 * rhs.m31 + self.m24 * rhs.m41;
        let m22 = self.m21 * rhs.m12 + self.m22 * rhs.m22 + self.m23 * rhs.m32 + self.m24 * rhs.m42;
        let m23 = self.m21 * rhs.m13 + self.m22 * rhs.m23 + self.m23 * rhs.m33 + self.m24 * rhs.m43;
        let m24 = self.m21 * rhs.m14 + self.m22 * rhs.m24 + self.m23 * rhs.m34 + self.m24 * rhs.m44;

        let m31 = self.m31 * rhs.m11 + self.m32 * rhs.m21 + self.m33 * rhs.m31 + self.m34 * rhs.m41;
        let m32 = self.m31 * rhs.m12 + self.m32 * rhs.m22 + self.m33 * rhs.m32 + self.m34 * rhs.m42;
        let m33 = self.m31 * rhs.m13 + self.m32 * rhs.m23 + self.m33 * rhs.m33 + self.m34 * rhs.m43;
        let m34 = self.m31 * rhs.m14 + self.m32 * rhs.m24 + self.m33 * rhs.m34 + self.m34 * rhs.m44;

        let m41 = self.m41 * rhs.m11 + self.m42 * rhs.m21 + self.m43 * rhs.m31 + self.m44 * rhs.m41;
        let m42 = self.m41 * rhs.m12 + self.m42 * rhs.m22 + self.m43 * rhs.m32 + self.m44 * rhs.m42;
        let m43 = self.m41 * rhs.m13 + self.m42 * rhs.m23 + self.m43 * rhs.m33 + self.m44 * rhs.m43;
        let m44 = self.m41 * rhs.m14 + self.m42 * rhs.m24 + self.m43 * rhs.m34 + self.m44 * rhs.m44;

        self.m11 = m11;
        self.m12 = m12;
        self.m13 = m13;
        self.m14 = m14;

        self.m21 = m21;
        self.m22 = m22;
        self.m23 = m23;
        self.m24 = m24;

        self.m31 = m31;
        self.m32 = m32;
        self.m33 = m33;
        self.m34 = m34;

        self.m41 = m41;
        self.m42 = m42;
        self.m43 = m43;
        self.m44 = m44;
    }
}

impl Quaternion {
    pub const IDENTITY: Quaternion = Quaternion {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0
    };

    pub fn magnitude_sq(&self) -> f64 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }
    pub fn magnitude(&self) -> f64 {
        self.magnitude_sq().sqrt()
    }

    pub fn normalized(&self) -> Quaternion {
        let magnitude = self.magnitude_sq();

        if magnitude == 0.0 {
            panic!("Zero magnitude");
        }

        *self * (1.0 / magnitude.sqrt())
    }
}

impl Mul<f64> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Quaternion {
            w: self.w * rhs,
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs
        }
    }
}
impl MulAssign<f64> for Quaternion {
    fn mul_assign(&mut self, rhs: f64) {
        self.w *= rhs;
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Mul for Quaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Quaternion {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w
        }
    }
}
impl MulAssign for Quaternion {
    fn mul_assign(&mut self, rhs: Self) {
        let w = self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z;
        let x = self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y;
        let y = self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x;
        let z = self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w;

        self.w = w;
        self.x = x;
        self.y = y;
        self.z = z;
    }
}