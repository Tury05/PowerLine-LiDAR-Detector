use arrsac::Arrsac;
use rand::distributions::Uniform;
use rand::{distributions::Distribution, Rng, SeedableRng};
use rand_xoshiro::Xoshiro256PlusPlus;
use sample_consensus::{Consensus, Estimator, Model};

#[derive(Debug, Clone, Copy)]
struct Vector2<T> {
    x: T,
    y: T,
}

impl Vector2<f64> {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y
    }
    fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    fn normalize(&self) -> Self {
        let v_norm = self.norm();
        Self {
            x: self.x / v_norm,
            y: self.y / v_norm,
        }
    }
}

impl core::ops::Mul<Vector2<f64>> for f64 {
    type Output = Vector2<f64>;
    fn mul(self, rhs: Vector2<f64>) -> Self::Output {
        Vector2 {
            x: self * rhs.x,
            y: self * rhs.y,
        }
    }
}

impl core::ops::Add for Vector2<f64> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

#[derive(Debug)]
struct Line {
    norm: Vector2<f64>,
    c: f64,
}

impl Model<Vector2<f64>> for Line {
    fn residual(&self, point: &Vector2<f64>) -> f64 {
        (self.norm.dot(point) + self.c).abs()
    }
}

struct LineEstimator;

impl Estimator<Vector2<f64>> for LineEstimator {
    type Model = Line;
    type ModelIter = std::iter::Once<Line>;
    const MIN_SAMPLES: usize = 2;

    fn estimate<I>(&self, mut data: I) -> Self::ModelIter
    where
        I: Iterator<Item = Vector2<f64>> + Clone,
    {
        let a = data.next().unwrap();
        let b = data.next().unwrap();
        let norm = Vector2::new(a.y - b.y, b.x - a.x).normalize();
        let c = -norm.dot(&b);
        std::iter::once(Line { norm, c })
    }
}

