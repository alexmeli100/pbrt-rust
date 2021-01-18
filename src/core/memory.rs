use std::ops::{Index, IndexMut};

pub type BlockedArray<T> = BlockedArrayX<T, 2>;

pub struct BlockedArrayX<T, const LogBlockSize: usize> {
    data    : Vec<T>,
    ures    : usize,
    vres    : usize,
    ublocks : usize,
}

impl<T: num::Zero + Copy, const LogBlockSize: usize> BlockedArrayX<T, {LogBlockSize}>
{
    pub fn new(ures: usize, vres: usize, data: Option<&[T]>) -> Self {
        let mut arr = Self {
            ures,
            vres,
            ublocks: 0,
            data: Vec::new()
        };

        let rures = arr.roundup(ures);
        let rvres = arr.roundup(vres);

        arr.ublocks = rures >> LogBlockSize;

        let mut d = vec![T::zero(); rures * rvres];

        if let Some(ref block) = data {
            for v in 0..vres {
                for u in 0.. ures {
                    arr[(u, v)] = block[v * ures + u];
                }
            }
        }

        arr.data = d;

        arr
    }

    fn roundup(&self, x: usize) -> usize {
        (x + self.block_size() - 1) & !(self.block_size() - 1)
    }


    pub const fn block_size(&self) -> usize {
        LogBlockSize
    }

    pub fn block(&self, a: usize) -> usize {
        a >> LogBlockSize
    }

    pub fn offset(&self, a: usize) -> usize {
        a & (self.block_size() - 1)
    }

    pub fn ures(&self) -> usize { self.ures }
    pub fn vres(&self) -> usize { self.vres }
}

impl<T: num::Zero + Copy, const LogBlockSize: usize> Index<(usize, usize)> for BlockedArrayX<T, {LogBlockSize}> {
    type Output = T;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        let (u, v) = index;
        let (bu, bv) = (self.block(u), self.block(v));
        let (ou, ov) = (self.offset(u), self.offset(v));
        let mut offset = self.block_size() * self.block_size() * (self.ublocks * bv + bu);
        offset += self.block_size() * ov + ou;

        &self.data[offset]
    }
}

impl<T: num::Zero + Copy, const LogBlockSize: usize> IndexMut<(usize, usize)> for BlockedArrayX<T, {LogBlockSize}> {

    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        let (u, v) = index;
        let (bu, bv) = (self.block(u), self.block(v));
        let (ou, ov) = (self.offset(u), self.offset(v));
        let mut offset = self.block_size() * self.block_size() * (self.ublocks * bv + bu);
        offset += self.block_size() * ov + ou;

        &mut self.data[offset]
    }
}
