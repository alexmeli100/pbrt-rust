use hexf::*;
use crate::core::pbrt::Float;

pub const FLOAT_ONE_MINUS_EPSILON   : Float = hexf32!("0x1.fffffep-1");
pub const ONE_MINUS_EPSILON         : Float = FLOAT_ONE_MINUS_EPSILON;
pub const PCG32_DEFAULT_STATE       : u64 = 0x853c_49e6_748f_ea9b;
pub const PCG32_DEFAULT_STREAM      : u64 = 0xda3e_39cb_94b9_5bdb;
pub const PCG32_MULT                : u64 = 0x5851_f42d_4c95_7f2d;

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
        let oldstate: u64 = self.state;
        let (mul, _) = oldstate.overflowing_mul(PCG32_MULT);
        let (add, _) = mul.overflowing_add(self.inc);
        self.state = add;
        let (shr, _) = oldstate.overflowing_shr(18);
        let combine = shr ^ oldstate;
        let (shr, _) = combine.overflowing_shr(27);
        let xorshifted: u32 = shr as u32;
        let (shr, _) = oldstate.overflowing_shr(59);
        let rot: u32 = shr as u32;
        let (shr, _overflow) = xorshifted.overflowing_shr(rot);
        let neg = !rot;
        let (add, _) = neg.overflowing_add(1_u32);
        let (shl, _) = xorshifted.overflowing_shl(add & 31);
        shr | shl
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

    pub fn set_sequence(&mut self, initseq: u64) {
        self.state = 0;
        let (shl, _) = initseq.overflowing_shl(1);
        self.inc = shl | 1;
        self.uniform_int32();
        let (add, _) = self.state.overflowing_add(PCG32_DEFAULT_STATE);
        self.state = add;
        self.uniform_int32();
    }
}

