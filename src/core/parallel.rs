use std::sync::atomic::{AtomicU32, Ordering};
use crate::core::pbrt::{Float, float_to_bits, bits_to_float};
use crossbeam::epoch::Atomic;

pub struct AtomicFloat {
    bits: AtomicU32
}

impl AtomicFloat {
    pub fn new(v: Float) -> Self {
        Self {
            bits: AtomicU32::new(float_to_bits(v))
        }
    }

    pub fn add(&self, v: Float) {
        let mut oldbits = self.bits.load(Ordering::Relaxed);

        loop {
            let newbits = float_to_bits(bits_to_float(oldbits));

            match self.bits.compare_exchange_weak(oldbits, newbits, Ordering::SeqCst, Ordering::Relaxed) {
                Ok(_) => break,
                Err(x) => oldbits = x
            }
        }
    }
}

impl Default for AtomicFloat {
    fn default() -> Self {
        Self {
            bits: AtomicU32::new(0)
        }
    }
}

impl From<AtomicFloat> for f32 {
    fn from(a: AtomicFloat) -> Self {
        f32::from_bits(a.bits.load(Ordering::Relaxed))
    }
}

impl From<AtomicFloat> for f64 {
    fn from(a: AtomicFloat) -> Self {
        f64::from_bits(a.bits.load(Ordering::Relaxed) as u64)
    }
}

impl Clone for AtomicFloat {
    fn clone(&self) -> Self {
        let bits = self.bits.load(Ordering::SeqCst);

        Self {
            bits: AtomicU32::new(bits)
        }
    }
}