use hexf::*;
use crate::core::pbrt::Float;

pub const ONE_MINUS_EPSILON: Float = hexf32!("0x1.fffffep-1");
pub const PCG32_DEFAULT_STATE: u64 = 0x853c_49e6_748f_ea9b;
pub const PCG32_DEFAULT_STREAM: u64 = 0xda3e_39cb_94b9_5bdb;
pub const PCG32_MULT: u64 = 0x5851_f42d_4c95_7f2d;

#[derive(Debug, Copy, Clone)]
pub struct RNG {
    state: u64,
    inc  : u64
}

impl Default for RNG {
    fn default() -> Self {
        Self {
            state: PCG32_DEFAULT_STATE,
            inc: PCG32_DEFAULT_STREAM
        }
    }
}

impl RNG {
    pub fn new(sequence_index: u64) -> Self {
        let mut rng = RNG::default();
        rng.set_sequence(sequence_index);

        rng
    }

    pub fn uniform_int32(&mut self) -> u32 {
        let old_state = self.state;
        self.state = old_state * PCG32_MULT + self.inc;
        let xor_shifted = (((old_state >> 18) ^ old_state) >> 27) as u32;
        let rot = (old_state >> 59) as u32;

        (xor_shifted >> rot) | (xor_shifted << ((!rot + 1) & 31))
    }

    pub fn uniform_int32_2(&mut self, b: u32) -> u32 {
        let threshold = (!b + 1) % b;

        loop {
            let r = self.uniform_int32();

            if r >= threshold {
                return r % b;
            }
        }
    }

    pub fn uniform_float(&mut self) -> Float {
        ONE_MINUS_EPSILON.min(self.uniform_int32() as Float * hexf32!("0x1.0p-32"))
    }

    pub fn set_sequence(&mut self, init_seq: u64) {
        self.state = 0;
        self.inc = (init_seq << 1) | 1;
        self.uniform_int32();
        self.state += PCG32_DEFAULT_STATE;
        self.uniform_int32();
    }
}

