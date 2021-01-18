use typed_arena::Arena;
use rayon::prelude::*;
use log::{info, warn};
use crossbeam::crossbeam_channel::bounded;
use crate::core::pbrt::Float;
use crate::core::primitive::{Primitive, Primitives};
use crate::core::geometry::bounds::Bounds3f;
use crate::core::geometry::point::Point3f;
use std::cmp::Ordering::Equal;
use crate::core::geometry::vector::Vector3f;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::SurfaceInteraction;
use crate::core::paramset::ParamSet;
use crate::core::light::AreaLights;
use crate::core::material::{Materials, TransportMode};
use bumpalo::Bump;

const NBUCKETS: usize = 12;
const MORTON_BITS: usize = 10;
const MORTON_SCALE: usize = 1 << MORTON_BITS;

#[derive(Debug, Default, Copy, Clone)]
struct BucketInfo {
    count   : usize,
    bounds  : Bounds3f
}

#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
struct MortonPrimitive {
    primitive_index : usize,
    morton_code     : u32
}

#[derive(Debug, Default, Copy, Clone)]
struct BVHPrimitiveInfo {
    primitive_number: usize,
    bounds          : Bounds3f,
    centroid        : Point3f
}

impl BVHPrimitiveInfo {
    fn new(primitive_number: usize, bounds: &Bounds3f) -> Self {
        Self {
            primitive_number,
            bounds: *bounds,
            centroid: bounds.p_min * 0.5 + bounds.p_max * 0.5
        }
    }
}

#[derive(Debug, Default)]
struct LBVHTreelet<'a> {
    start_index : usize,
    n_primitives: usize,
    build_nodes : &'a mut [BVHBuildNode<'a>],
    treelet_root: Option<&'a BVHBuildNode<'a>>
}

impl<'a> LBVHTreelet<'a> {
    fn new(start_index: usize, n_primitives: usize, build_nodes: &'a mut [BVHBuildNode<'a>]) -> Self {
        Self {
            start_index,
            n_primitives,
            build_nodes,
            treelet_root: Default::default()
        }
    }
}

pub struct LinearBVHNode {
    bounds      : Bounds3f,
    n_primitives: u16,
    offset      : u32,
    axis        : u8,
    pad         : u8
}

#[derive(Debug, Default, Copy, Clone)]
struct BVHBuildNode<'a> {
    bounds              : Bounds3f,
    left                : Option<&'a BVHBuildNode<'a>>,
    right               : Option<&'a BVHBuildNode<'a>>,
    splitaxis           : usize,
    first_prim_offset   : usize,
    n_primitives        : usize
}

impl<'a> BVHBuildNode<'a> {
    fn init_leaf(&mut self, first: usize, n: usize, b: &Bounds3f) {
        self.first_prim_offset = first;
        self.n_primitives = n;
        self.bounds = *b;
        self.left = None;
        self.right = None;
    }

    fn init_interior(&mut self, axis: usize, left: &'a BVHBuildNode<'a>, right: &'a BVHBuildNode<'a>) {
        self.left = Some(left);
        self.right = Some(right);
        self.bounds = left.bounds.union_bounds(&right.bounds);
        self.splitaxis = axis;
        self.n_primitives = 0;
    }
}

#[derive(Debug, Ord, PartialOrd, Eq, PartialEq, Copy, Clone)]
pub enum SplitMethod {
    SAH,
    HLBVH,
    Middle,
    EqualCounts
}

pub struct BVHAccel {
    pub max_prims   : usize,
    pub split_method: SplitMethod,
    pub primitives  : Vec<Arc<Primitives>>,
    pub nodes       : Vec<LinearBVHNode>
}

impl BVHAccel {
    pub fn new(p: Vec<Arc<Primitives>>, max_prims: usize, split_method: SplitMethod) -> Self {
        let b = Self {
            primitives: p,
            split_method,
            max_prims,
            nodes: vec![]
        };
        // Build BVH from primitives

        // Initialize primitiveInfo array for primitives
        let mut primitive_info: Vec<BVHPrimitiveInfo> = Vec::with_capacity(b.primitives.len());
        for i in 0..b.primitives.len() {
            primitive_info[i] = BVHPrimitiveInfo::new(i, &b.primitives[i].world_bound()) ;
        }

        let arena = Arena::with_capacity(1024 * 1024);
        let mut total_nodes = 0;
        let mut ordered_prims = Vec::with_capacity(b.primitives.len());

        let root = match split_method {
            SplitMethod::HLBVH => b.hlbvh_build(&arena, &primitive_info, &mut total_nodes, &mut ordered_prims),
            _                  => b.recursive_build(&arena, &mut primitive_info, 0, b.primitives.len(), &mut total_nodes, &mut ordered_prims)
        };

        info!(
            "BVH created with {} nodes for {} primitives ({} MB), arena allocated {} MB",
            total_nodes,
            b.primitives.len(),
            (total_nodes * std::mem::size_of::<LinearBVHNode>()) as f32 / (1024.0 * 1024.0),
            arena.len() as f32 / (1024.0 * 1024.0));

        //b.primitives = ordered_prims;
        let mut nodes = Vec::with_capacity(total_nodes);
        //b.nodes = nodes;
        let mut offset = 0;
        BVHAccel::flatten_bvhtree(&mut nodes, root, &mut offset);
        assert_eq!(total_nodes, offset);

        Self {
            max_prims,
            split_method,
            primitives: ordered_prims,
            nodes
        }
    }

    fn recursive_build<'a>(
        &self,
        arena: &'a Arena<BVHBuildNode<'a>>,
        primitive_info: &mut [BVHPrimitiveInfo],
        start: usize,
        end: usize,
        total_nodes: &mut usize,
        ordered_prims: &mut Vec<Arc<Primitives>>
    ) -> &'a BVHBuildNode<'a>
    {
        let node = arena.alloc(Default::default());
        *total_nodes += 1;
        let mut bounds = Bounds3f::default();

        for i in start..end {
            bounds = bounds.union_bounds(&primitive_info[i].bounds);
        }

        let nprims = end - start;

        if nprims == 1 {
            let offset = ordered_prims.len();

            for p  in &primitive_info[start..end] {
                let prim_num = p.primitive_number;
                ordered_prims.push(self.primitives[prim_num].clone());
            }

            node.init_leaf(offset, nprims, &bounds);
            return node;
        } else {
            // Compute bound of primitive centroids, choose split dimension dim
            let mut centroid_bounds = Bounds3f::default();

            for i in start..end {
                centroid_bounds = centroid_bounds.union_point(&primitive_info[i].centroid);
            }

            let dim = centroid_bounds.maximum_extent();

            // Partition primitives into two sets and build children
            let mut mid = (start + end) / 2;

            if centroid_bounds.p_max[dim] == centroid_bounds.p_min[dim] {
                let offset = ordered_prims.len();

                for p in &primitive_info[start..end] {
                    let prim_num = p.primitive_number;
                    ordered_prims.push(self.primitives[prim_num].clone());
                }

                node.init_leaf(offset, nprims, &bounds);
                return node;
            } else {
                if self.split_method == SplitMethod::Middle {
                    mid = self.split_middle(dim, &centroid_bounds, primitive_info)
                }

                match self.split_method {
                    SplitMethod::Middle      => if !(mid != start && mid != end) { mid = self.split_equal(dim, start, end, primitive_info); } else {  },
                    SplitMethod::EqualCounts => mid = self.split_equal(dim, start, end, primitive_info),
                    _ => {
                        let (m, create_leaf) = self.split_sah(&bounds, &centroid_bounds, dim, nprims, start, end, primitive_info);

                        if create_leaf {
                            let offset = ordered_prims.len();

                            for p in &primitive_info[start..end] {
                                let prim_num = p.primitive_number;
                                ordered_prims.push(self.primitives[prim_num].clone());
                            }

                            node.init_leaf(offset, nprims, &bounds);
                            return node;
                        } else {
                            mid = m
                        }
                    },
                };

                node.init_interior(
                    dim,
                    self.recursive_build(arena, primitive_info, start, mid, total_nodes, ordered_prims),
                    self.recursive_build(arena, primitive_info, mid, end, total_nodes, ordered_prims)
                );
            }
        }

        node
    }

    fn split_middle(&self, dim: usize, centroid_bounds: &Bounds3f, primitive_info: &mut [BVHPrimitiveInfo]) -> usize {
        let pmid = (centroid_bounds.p_min[dim] + centroid_bounds.p_max[dim]) / 2.0;

        primitive_info.iter_mut().partition_in_place(|p|  p.centroid[dim] < pmid)
    }

    fn split_equal(&self, dim: usize, start: usize, end: usize, primitive_info: &mut [BVHPrimitiveInfo]) -> usize {
        let mid = (start + end) / 2;

        primitive_info.select_nth_unstable_by(mid, |a, b| {
            a.centroid[dim].partial_cmp(&b.centroid[dim]).unwrap_or(Equal)
        });

        mid
    }

    fn split_sah(
        &self, bounds: &Bounds3f, centroid_bounds: &Bounds3f, dim: usize, nprims: usize,
        start: usize, end: usize, primitive_info: &mut [BVHPrimitiveInfo]) -> (usize, bool) {
        // partition primitives using approximate SAH
        if nprims <= 4 {
            // partition primitives into equally sized subsets
            (self.split_equal(dim, start, end, primitive_info), false)
        } else {
            // Allocate BucketInfo for SAH partition buckets
            let mut buckets = [BucketInfo::default(); NBUCKETS];

            // Initialize BucketInfo for SAH partition buckets
            primitive_info.iter().for_each(|p| {
                let mut b = (NBUCKETS as Float * centroid_bounds.offset(&p.centroid)[dim]) as usize;

                if b == NBUCKETS { b = NBUCKETS - 1}
                buckets[b].count += 1;
                buckets[b].bounds = buckets[b].bounds.union_bounds(&p.bounds);
            });

            // Compute cost for splitting after each bucket
            let mut cost: [Float; NBUCKETS - 1] = [0.0; NBUCKETS - 1];

            for i in 0..NBUCKETS-1 {
                let (mut b0, mut b1) = (Bounds3f::default(), Bounds3f::default());
                let (mut count0, mut count1) = (0, 0);

                for j in 0..=i {
                    b0 = b0.union_bounds(&buckets[j].bounds);
                    count0 += buckets[j].count
                }

                for j in i+1..NBUCKETS {
                    b1 = b1.union_bounds(&buckets[j].bounds);
                    count1 += buckets[j].count;
                }

                cost[i] = 1.0 + (count0 as Float * b0.surface_area() + count1 as Float * b1.surface_area()) / bounds.surface_area();
            }

            // Find bucket to split at that minimizes SAH metric
            let (min_cost_split_bucket, min_cost) = cost[0..NBUCKETS-1].iter()
                .enumerate()
                .min_by(|(_, c1), (_, c2)| c1.partial_cmp(c2).unwrap_or(Equal))
                .unwrap();

            // Split primitive at selected SAH bucket
            let leaf_cost = nprims as Float;

            if nprims > self.max_prims || *min_cost < leaf_cost {
               let pmid =  primitive_info.iter_mut().partition_in_place(|pi| {
                    let mut b = (NBUCKETS as Float * centroid_bounds.offset(&pi.centroid)[dim]) as usize;

                    if b == NBUCKETS { b = NBUCKETS - 1; }

                    b <= min_cost_split_bucket

                });

                return (pmid, false);
            }

            (0, true)
        }
    }

    fn hlbvh_build<'a>(
        &'a self,
        arena: &'a Arena<BVHBuildNode<'a>>,
        primitive_info: &Vec<BVHPrimitiveInfo>,
        total_nodes: &mut usize,
        ordered_prims: &mut Vec<Arc<Primitives>>
    ) -> &'a BVHBuildNode<'a>
    {
        // Compute bounding box of all primitve centroids
        let mut bounds = Bounds3f::default();

        for pi in primitive_info.iter() {
            bounds = bounds.union_point(&pi.centroid);
        }

        let mut morton_prims = vec![MortonPrimitive::default(); primitive_info.len()];
        morton_prims
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, mp)| {
                mp.primitive_index = primitive_info[i].primitive_number;
                let centroid_offset = bounds.offset(&primitive_info[i].centroid);
                mp.morton_code = encode_morton3(&(centroid_offset * MORTON_SCALE as Float));
            });

        radix_sort(&mut morton_prims);

        // Create LBVHs for treelets in parallel

        // Find intervals of primitives for each treelet
        let mut treelets_tobuild: Vec<LBVHTreelet<'a>> = Vec::new();
        let mut start = 0;

        for end in 1..=morton_prims.len() {
            let mask = 0x3ffc0000;

            if end == morton_prims.len() ||
                ((morton_prims[start].morton_code & mask) !=
                morton_prims[end].morton_code & mask) {
                // Add entry to treelets_tobuild for this treelet
                let n_primitives = end - start;
                let max_bvh_nodes = 2 * n_primitives - 1;
                let nodes = arena.alloc_extend(vec![Default::default(); max_bvh_nodes].into_iter());
                treelets_tobuild.push(LBVHTreelet::new(start, n_primitives, nodes));
                start = end;
            }
        }

        let atomic_total = AtomicUsize::new(0);
        let prims_offset = AtomicUsize::new(0);

        //ordered_primes.resize_with(self.primitives.len(), || Arc::new(Default::default()));
        let (s, r) = bounded(ordered_prims.len());

        treelets_tobuild
            .par_iter_mut()
            .for_each(|tr: &mut LBVHTreelet<'a>| {
                let mut nodes_created = 0;
                let first_bit_index = 29 - 12;

                let nodes = self.emit_lbvh(
                    tr.build_nodes.as_mut_ptr(),
                    primitive_info,
                    &morton_prims[tr.start_index..],
                    tr.n_primitives,
                    &mut nodes_created,
                    s.clone(),
                    &prims_offset,
                    first_bit_index,
                );
                tr.treelet_root = Some(nodes);
                atomic_total.fetch_add(nodes_created, Ordering::SeqCst);
            });

        for _ in 0..ordered_prims.len() {
            let (off, p_off) = r.recv().unwrap();
            ordered_prims[off] = self.primitives[p_off].clone()
        }

        *total_nodes += atomic_total.load(Ordering::SeqCst);

        // Create and return SAH BVH from LBVH treelets
        let mut finished_treelets: Vec<&'a BVHBuildNode<'a>> = Vec::with_capacity(treelets_tobuild.len());

        for treelet in treelets_tobuild.into_iter() {
            finished_treelets.push(treelet.treelet_root.unwrap());
        }

        let end = finished_treelets.len();
        self.build_upper_sah(arena, &mut finished_treelets, 0, end, total_nodes)
    }

    fn emit_lbvh<'a>(
        &'a self, nodes: *mut BVHBuildNode<'a>, primitive_info: &Vec<BVHPrimitiveInfo>,
        morton_prims: &[MortonPrimitive], n_primitives: usize, total_nodes: &mut usize,
        sender: crossbeam::Sender<(usize, usize)>, ordered_prims_offset: &AtomicUsize, bit_index: isize,
    ) -> &'a BVHBuildNode<'a>
    {
        if bit_index == -1 || n_primitives < self.max_prims {
            // Create and return leaf node of LBVH treelet

            let node: &mut BVHBuildNode;

            unsafe {
                node = &mut *nodes.add(*total_nodes);
            }
            *total_nodes += 1;

            let mut bounds: Bounds3f = Default::default();
            let first_prim_offset = ordered_prims_offset.fetch_add(n_primitives, Ordering::SeqCst);

            for (i, mp) in morton_prims.iter().enumerate() {
                let primitive_index = mp.primitive_index;
                sender.send((first_prim_offset+i, primitive_index)).unwrap();
                //ordered_prims[first_prim_offset + i] = self.primitives[primitive_index].clone();
                bounds = bounds.union_bounds(&primitive_info[primitive_index].bounds);
            }

            node.init_leaf(first_prim_offset, n_primitives, &bounds);
            node
        } else {
            let mask = 1 << bit_index;

            // Advance to next subtree level if there's no LBVH split for this bit
            if (morton_prims[0].morton_code & mask) == (morton_prims[n_primitives - 1].morton_code & mask) {
                return self.emit_lbvh(nodes, primitive_info, morton_prims, n_primitives, total_nodes, sender.clone(), ordered_prims_offset, bit_index - 1);
            }

            // Find LBVH split point for this dimension
            let mut seach_start = 0;
            let mut search_end = n_primitives - 1;

            while seach_start + 1 != search_end {
                let mid = (seach_start + search_end) / 2;

                if (morton_prims[seach_start].morton_code & mask) == morton_prims[mid].morton_code & mask {
                    seach_start = mid;
                } else {
                    search_end = mid;
                }
            }

            let split_offset = search_end;

            // Create and return interior LBVH node
            let node: &'a mut BVHBuildNode;

            unsafe {
                node = &mut *nodes.add(*total_nodes);
            }

            *total_nodes += 1;

            let left = self.emit_lbvh
            (
                nodes,
                primitive_info,
                &morton_prims[0..split_offset],
                split_offset,
                total_nodes,
                sender.clone(),
                ordered_prims_offset,
                bit_index - 1,
            );

            let right = self.emit_lbvh(
                nodes,
                primitive_info,
                &morton_prims[split_offset..],
                n_primitives - split_offset,
                total_nodes,
                sender.clone(),
                ordered_prims_offset,
                bit_index - 1,
            );

            let axis = bit_index % 3;
            node.init_interior(axis as usize, left, right);

            node
        }
    }

    fn build_upper_sah<'a>(
        &self, arena: &'a Arena<BVHBuildNode<'a>>, treelet_roots: &mut [&'a BVHBuildNode<'a>],
        start: usize, end: usize, total_nodes: &mut usize) -> &'a BVHBuildNode<'a> {
        let n_nodes = end - start;
        if n_nodes == 1 { return treelet_roots[start]; }
        *total_nodes += 1;
        let node = arena.alloc(Default::default());

        let mut bounds = Bounds3f::default();
        let mut centroid_bounds = Bounds3f::default();

        for tr in treelet_roots.iter() {
            bounds = bounds.union_bounds(&tr.bounds);
            let centroid = (tr.bounds.p_min + tr.bounds.p_max) * 0.5;
            centroid_bounds = centroid_bounds.union_point(&centroid);
        }

        let dim = centroid_bounds.maximum_extent();

        // Allocate BucketInfo for SAH partition buckets
        let mut buckets = [BucketInfo::default(); NBUCKETS];

        // Initialize BucketInfo for SAH partition buckets
        treelet_roots.iter().for_each(|tr| {
            let centroid = (tr.bounds.p_min[dim] + tr.bounds.p_max[dim]) * 0.5;
            let mut b = (NBUCKETS as Float * ((centroid - centroid_bounds.p_min[dim]) / (centroid_bounds.p_max[dim] - centroid_bounds.p_min[dim]))) as usize;

            if b == NBUCKETS { b = NBUCKETS - 1}
            buckets[b].count += 1;
            buckets[b].bounds = buckets[b].bounds.union_bounds(&tr.bounds);
        });

        // Compute cost for splitting after each bucket
        let mut cost: [Float; NBUCKETS - 1] = [0.0; NBUCKETS - 1];

        for i in 0..NBUCKETS-1 {
            let (mut b0, mut b1) = (Bounds3f::default(), Bounds3f::default());
            let (mut count0, mut count1) = (0, 0);

            for j in 0..=i {
                b0 = b0.union_bounds(&buckets[j].bounds);
                count0 += buckets[j].count
            }

            for j in i+1..NBUCKETS {
                b1 = b1.union_bounds(&buckets[j].bounds);
                count1 += buckets[j].count;
            }

            cost[i] = 0.125 + (count0 as Float * b0.surface_area() + count1 as Float * b1.surface_area()) / bounds.surface_area();
        }

        // Find bucket to split at that minimizes SAH metric
        let (min_cost_split_bucket, min_cost) = cost[0..NBUCKETS-1].iter()
            .enumerate()
            .min_by(|(_, c1), (_, c2)| c1.partial_cmp(c2).unwrap_or(Equal))
            .unwrap();


        let pmid =  treelet_roots.iter_mut().partition_in_place(|tr| {
            let centroid = (tr.bounds.p_min[dim] + tr.bounds.p_max[dim]) * 0.5;
            let mut b = (NBUCKETS as Float * ((centroid - centroid_bounds.p_min[dim]) / (centroid_bounds.p_max[dim] - centroid_bounds.p_min[dim]))) as usize;

            if b == NBUCKETS { b = NBUCKETS - 1; }

            b <= min_cost_split_bucket

        });

        node.init_interior(
            dim,
            self.build_upper_sah(arena, treelet_roots, start, pmid, total_nodes),
            self.build_upper_sah(arena, treelet_roots, pmid, end, total_nodes)
        );

        node
    }

    fn flatten_bvhtree<'a>(nodes: &mut [LinearBVHNode], node: &BVHBuildNode<'a>, offset: &mut usize) -> usize {
        let my_offset = *offset;
        *offset += 1;

        if node.n_primitives > 0 {
            nodes[my_offset] = LinearBVHNode {
                bounds :node.bounds,
                n_primitives: node.n_primitives as u16,
                offset: node.first_prim_offset as u32,
                axis: 0,
                pad: 0_u8
            };
        } else {
            if let Some(ref l) = node.left {
                BVHAccel::flatten_bvhtree(nodes, l, offset);
            }

            if let Some(ref r) = node.right {
                let linear_node = LinearBVHNode {
                    bounds: node.bounds,
                    n_primitives: 0,
                    offset: BVHAccel::flatten_bvhtree(nodes, r, offset) as u32,
                    axis: node.splitaxis as u8,
                    pad: 0_u8
                };

                nodes[my_offset] = linear_node
            }
        }

        my_offset
    }
}

impl Primitive for BVHAccel {
    fn world_bound(&self) -> Bounds3f {
        if self.nodes.is_empty() {
            return Bounds3f::default()
        }

        self.nodes[0].bounds
    }

    fn intersect(&self, r: &mut Ray, isect: &mut SurfaceInteraction) -> bool {
        let mut hit = false;
        let inv_dir = Vector3f::new(1.0 / r.d.x, 1.0 / r.d.y, 1.0 / r.d.z);
        let dir_isneg: [usize; 3] = [
            (inv_dir.x < 0.0) as usize,
            (inv_dir.y < 0.0) as usize,
            (inv_dir.z < 0.0) as usize
        ];

        // follow ray through BVH nodes to find primitive intersections
        let mut to_visitoffset = 0;
        let mut currentnode_index = 0;
        let mut nodes_tovisit = vec![0; 64];

        loop {
            let node = &self.nodes[currentnode_index];

            if node.bounds.intersect_p2(r, &inv_dir, dir_isneg) {
                if node.n_primitives > 0 {
                    // Intersect ray with primitives in leaf BVH node;
                    for i in 0..node.n_primitives {
                        if self.primitives[node.offset as usize + i as usize].intersect(r, isect) { hit = true; }
                        if to_visitoffset == 0 { break; }
                        to_visitoffset -= 1;
                        currentnode_index = nodes_tovisit[to_visitoffset];
                    }
                } else {
                    // Put far BVH node on nodes_tovisit stack, advance to near node
                    if dir_isneg[node.axis as usize] != 0 {
                        nodes_tovisit[to_visitoffset] = currentnode_index + 1;
                        to_visitoffset += 1;
                        currentnode_index = node.offset as usize;
                    } else {
                        nodes_tovisit[to_visitoffset] = node.offset as usize;
                        to_visitoffset += 1;
                        currentnode_index += 1;
                    }
                }
            } else {
                if to_visitoffset == 0 { break; }
                to_visitoffset -= 1;
                currentnode_index = nodes_tovisit[to_visitoffset];
            }
        }

        hit
    }

    fn intersect_p(&self, r: &mut Ray) -> bool {
        if self.nodes.is_empty() {
            return false;
        }

        let inv_dir = Vector3f::new(1.0 / r.d.x, 1.0 / r.d.y, 1.0 / r.d.z);
        let dir_isneg: [usize; 3] = [
            (inv_dir.x < 0.0) as usize,
            (inv_dir.y < 0.0) as usize,
            (inv_dir.z < 0.0) as usize
        ];

        // follow ray through BVH nodes to find primitive intersections
        let mut to_visitoffset = 0;
        let mut currentnode_index = 0;
        let mut nodes_tovisit = vec![0; 64];

        loop {
            let node = &self.nodes[currentnode_index];

            if node.bounds.intersect_p2(r, &inv_dir, dir_isneg) {
                // Process BVH node for traversal
                if node.n_primitives > 0 {
                    for i in 0..node.n_primitives {
                        if self.primitives[node.offset as usize + i as usize].intersect_p(r) {
                            return true;
                        }
                    }

                    if to_visitoffset == 0 { break; }
                    to_visitoffset -= 1;
                    currentnode_index = nodes_tovisit[to_visitoffset];

                } else {
                    // Put far BVH node on nodes_tovisit stack, advance to near node
                    if dir_isneg[node.axis as usize] != 0 {
                        // second child first
                        nodes_tovisit[to_visitoffset] = currentnode_index + 1;
                        to_visitoffset += 1;
                        currentnode_index = node.offset as usize;
                    } else {
                        nodes_tovisit[to_visitoffset] = node.offset as usize;
                        to_visitoffset += 1;
                        currentnode_index += 1;
                    }
                }
            } else {
                if to_visitoffset == 0 { break; }
                to_visitoffset -= 1;
                currentnode_index = nodes_tovisit[to_visitoffset];
            }
        }

        false
    }

    fn get_material(&self) -> Option<Arc<Materials>> {
        panic!("BVH::get_material method called; should have gone to GeometricPrimitive")
    }

    fn get_area_light(&self) -> Option<Arc<AreaLights>> {
        panic!("BVH::get_area_light method called; should have gone to GeometricPrimitive")
    }

    fn compute_scattering_functions<'a: 'a>(
        &self, _isect: &mut SurfaceInteraction<'a>, _arena: &'a Bump,
        _mode: TransportMode, _allow_multiple_lobes: bool) {
        panic!("BVH::get_compute_scattering_functions method called; should have gone to GeometricPrimitive")
    }
}


#[inline(always)]
fn left_shift3(n: u32) -> u32 {
    let mut x = n;

    if x == (1 << 10) { x -= 1; }

    x = (x | (x << 16)) & 0x30000ff;
    // x = ---- --98 ---- ---- ---- ---- 7654 3210
    x = (x | (x << 8)) & 0x300f00f;
    // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    x = (x | (x << 4)) & 0x30c30c3;
    // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    x = (x | (x << 2)) & 0x9249249;
    // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0

    x
}

#[inline(always)]
fn encode_morton3(v: &Vector3f) -> u32 {
    (left_shift3(v.z as u32) << 2) | (left_shift3(v.y as u32) << 1) | left_shift3(v.x as u32)
}

fn radix_sort(v: &mut [MortonPrimitive]) {
    let mut temp_vector: Vec<MortonPrimitive> = Vec::with_capacity(v.len());
    const BITS_PER_PASS: usize = 6;
    let nbits = 30;
    let npasses = nbits / BITS_PER_PASS;

    assert_eq!((nbits % BITS_PER_PASS), 0, "Radix sort bitsPerPass must evenly divide nBits");

    for pass in 0..npasses {
        // Perform one pass of radix sort, sorting bitsPerPass bits
        let low_bit = pass * BITS_PER_PASS;
        let in_vec: &mut [MortonPrimitive];
        let out_vec: &mut [MortonPrimitive];

        if (pass & 1) != 0 {
            in_vec = temp_vector.as_mut_slice();
            out_vec = v;
        } else {
            in_vec = v;
            out_vec = temp_vector.as_mut_slice();
        }

        // Count number of zero bits in array for current radix sort bit
        const NBUCKETS: usize = 1 << BITS_PER_PASS;
        let mut bucket_count= vec![0; NBUCKETS];
        let bit_mask = ( 1 << BITS_PER_PASS) - 1;

        for mp in in_vec.iter() {
            let bucket = (mp.morton_code >> low_bit) & bit_mask;
            bucket_count[bucket as usize] += 1;
        }

        // Count starting index in output array for each bucket
        let mut out_index = vec![0; NBUCKETS];
        out_index[0] = 0;

        for i in 1..NBUCKETS {
            out_index[i] = out_index[i - 1] + bucket_count[i - 1];
        }

        // store sorted values in output array
        for mp in in_vec.iter() {
            let bucket = (mp.morton_code >> low_bit) & bit_mask;
            out_vec[out_index[bucket as usize]] = *mp;
            out_index[bucket as usize] += 1;
        }

    }

    if (npasses & 1) != 0 {
        v.swap_with_slice(&mut temp_vector[0..]);
    }
}

pub fn create_bvh_accelerator(prims: Vec<Arc<Primitives>>, ps: &ParamSet) -> Arc<Primitives> {
    let split_name = ps.find_one_string("splitmethod", "sah".to_owned());
    let split_method= match split_name.as_str() {
        "sah"    => SplitMethod::SAH,
        "hlbvh"  => SplitMethod::HLBVH,
        "middle" => SplitMethod::Middle,
        "equal"  => SplitMethod::EqualCounts,
        _  => {
            warn!("BVH split method \"{}\" unknown. Using \"sah\".", split_name);
            SplitMethod::SAH
        }
    };

    let max_prims = ps.find_one_int("maxnodeprims", 4);
    let bvh = BVHAccel::new(prims, max_prims as usize, split_method);

    Arc::new(bvh.into())
}