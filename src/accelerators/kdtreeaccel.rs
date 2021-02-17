use crate::core::pbrt::{Float, log2_int64, INFINITY};
use std::sync::Arc;
use crate::core::primitive::{Primitive, Primitives};
use crate::core::geometry::bounds::Bounds3f;
use crate::core::interaction::SurfaceInteraction;
use crate::core::geometry::ray::Ray;
use crate::core::geometry::vector::Vector3f;
use crate::core::paramset::ParamSet;
use crate::core::light::{Lights};
use crate::core::material::{Materials, TransportMode};
use bumpalo_herd::Member;

const MAX_TODO: usize = 64;

#[repr(C)]
union LeafData {
    split                   : Float,
    one_primitive           : i32,
    primtive_offset_indices : i32
}

#[repr(C)]
union ChildData {
    flags       : i32,
    nprims      : i32,
    above_child : i32
}

struct KdAccelNode {
    leaf    : LeafData,
    child   : ChildData,
}

impl KdAccelNode {
    fn init_interior(&mut self, axis: i32, ac: i32, s: Float) {
        self.leaf.split = s;
        self.child.flags = axis;
        let above_child: i32;

        unsafe {
            above_child = self.child.above_child;
        }
        self.child.above_child = above_child |  (ac << 2);
    }

    fn init_leaf(&mut self, prim_nums: &[usize], np: usize, primitive_indices: &mut Vec<usize>) {
        self.child.flags = 3;
        let nprims: i32;

        unsafe {
            nprims = self.child.nprims;
        }

        self.child.nprims = nprims | ((np as i32) << 2);

        match np {
            0 => self.leaf.one_primitive = 0,
            1 => self.leaf.one_primitive = prim_nums[0] as i32,
            _ => {
                self.leaf.primtive_offset_indices = primitive_indices.len() as i32;

                for i in 0..np {
                    primitive_indices.push(prim_nums[i as usize] as usize);
                }
            }
        }
    }

    fn split_pos(&self) -> Float {
        unsafe { self.leaf.split }
    }

    fn n_primitives(&self) -> i32 {
        unsafe { self.child.nprims }
    }

    fn split_axis(&self) -> i32 {
        unsafe { self.child.flags & 3}
    }

    fn is_leaf(&self) -> bool {
        unsafe { (self.child.flags & 3) == 3 }
    }

    fn above_child(&self) -> i32 {
        unsafe { self.child.above_child >> 2 }
    }
}

impl Default for KdAccelNode {
    fn default() -> Self {
        Self {
            child: ChildData { flags: 0_i32},
            leaf: LeafData { one_primitive: 0_i32}
        }
    }
}

#[derive(Debug, PartialOrd, PartialEq)]
pub enum EdgeType {
    Start,
    End
}

impl Default for EdgeType {
    fn default() -> Self {
        EdgeType::Start
    }
}

#[derive(Debug, Default)]
struct BoundEdge {
    t        : Float,
    prim_num : usize,
    edge_type: EdgeType
}

impl BoundEdge {
    fn new(t: Float, prim_num: usize, edge_type: EdgeType) -> Self {
        Self {
            t,
            prim_num,
            edge_type
        }
    }
}

#[derive(Default, Copy, Clone)]
struct KdToDo<'a> {
    node    : Option<&'a KdAccelNode>,
    idx     : usize,
    t_min   : Float,
    t_max   : Float
}

#[derive(Default)]
pub struct KdTreeAccel {
    isect_cost          : isize,
    traversal_cost      : isize,
    max_prims           : usize,
    empty_bonus         : Float,
    primtives           : Vec<Arc<Primitives>>,
    primitive_indices   : Vec<usize>,
    nodes               : Vec<KdAccelNode>,
    n_allocednodes      : usize,
    next_freenode       : usize,
    bounds              : Bounds3f
}

impl KdTreeAccel {
    pub fn new(
        p: Vec<Arc<Primitives>>, isect_cost: isize, traversal_cost: isize,
        empty_bonus: Float, max_prims: usize, max_depth: usize) -> Self {
        let mut kd = Self {
            isect_cost,
            traversal_cost,
            max_prims,
            empty_bonus,
            primtives: p,
            ..Default::default()
        };

        let mut depth = max_depth;

        if max_depth <= 0 {
            depth = ((8.0 + 1.3 * log2_int64(kd.primtives.len() as i64) as Float).round()) as usize;
        }

        // Compute bounds for kd-tree construction
        let mut prim_bounds = Vec::with_capacity(kd.primtives.len());
        let mut bounds = Bounds3f::default();

        kd.primtives
            .iter()
            .for_each(|p| {
                let b = p.world_bound();
                bounds = bounds.union_bounds(&b);
                prim_bounds.push(b)
            });

        kd.bounds = bounds;

        // Allocate working memory for kd-tree construction
        let mut edges: [Vec<BoundEdge>; 3] = [
            Vec::with_capacity(2 * kd.primtives.len()),
            Vec::with_capacity(2 * kd.primtives.len()),
            Vec::with_capacity(2 * kd.primtives.len())
        ];

        let mut prim0 = Vec::with_capacity(kd.primtives.len());
        let mut prim1 = Vec::with_capacity((max_depth + 1) * kd.primtives.len());

        // Initialize primitive_nums for kd-tree construction
        let mut prim_nums = Vec::with_capacity(kd.primtives.len());

        for i in 0..kd.primtives.len() {
            prim_nums.push(i);
        }

        kd.build_tree(
            0,
            &kd.bounds.clone(),
            &prim_bounds,
            &mut prim_nums,
            kd.primtives.len(),
            depth,
            &mut edges,
            &mut prim0[..],
            &mut prim1[..],
            0
        );

        kd
    }

    fn build_tree(
        &mut self, node_num: usize, node_bounds: &Bounds3f,
        all_prim_bounds: &Vec<Bounds3f>, prim_nums: &mut [usize],
        n_primitives: usize, depth: usize, edges: &mut [Vec<BoundEdge>; 3],
        prim0: &mut [usize], prim1: &mut [usize], bad_refines: isize
    ) {
        let mut bad_refines = bad_refines;
        // Get next free node from nodes array
        if self.next_freenode == self.n_allocednodes {
            let n_new_allocednodes = std::cmp::max(2 * self.n_allocednodes, 512);

            if self.n_allocednodes > 0 {
                self.nodes.resize_with(n_new_allocednodes, || KdAccelNode::default())

            } else {
                let mut n = Vec::with_capacity(n_new_allocednodes);

                for _ in 0..n_new_allocednodes {
                    n.push(KdAccelNode::default())
                }

                self.nodes = n;
            }

            self.n_allocednodes = n_new_allocednodes;
        }

        self.next_freenode += 1;

        // Initialize leaf node if termination criteria met
        if n_primitives <= self.max_prims || depth == 0 {
            self.nodes[node_num].init_leaf(prim_nums, n_primitives, &mut self.primitive_indices);
            return;
        }


        // Initialize interior node and continue recursion

        // choose split axis position for interior node
        let mut best_axis = -1_isize;
        let mut best_offset = -1_isize;
        let mut best_cost = INFINITY;
        let old_cost = self.isect_cost as Float * n_primitives as Float;
        let total_sa = node_bounds.surface_area();
        let inv_total_sa = 1.0 / total_sa;
        let d = node_bounds.p_max - node_bounds.p_min;

        // Choose which axis to split along
        let mut axis = node_bounds.maximum_extent();
        let mut retries = 0;

        loop {
            for i in 0..n_primitives {
                let pn = prim_nums[i];
                let bounds = &all_prim_bounds[pn];
                edges[axis][2 * i] = BoundEdge::new(bounds.p_min[axis], pn, EdgeType::Start);
                edges[axis][2 * i + 1] = BoundEdge::new(bounds.p_max[axis], pn, EdgeType::End);
            }

            // Sort edges for axis
            edges[axis].sort_unstable_by(|e0, e1| {
                if e0.t == e1.t {
                    e0.edge_type.partial_cmp(&e1.edge_type).unwrap()
                } else {
                    e0.t.partial_cmp(&e1.t).unwrap()
                }
            });

            // Conpute cost of all splits for axis to find best
            let mut nbelow = 0;
            let mut nabove = n_primitives;

            for i in 0..2*n_primitives {
                if edges[axis][i].edge_type == EdgeType::End {
                    nabove -= 1;
                }

                let edge_t = edges[axis][i].t;

                if edge_t > node_bounds.p_min[axis] && edge_t < node_bounds.p_max[axis] {
                    // Compute cost for split at ith edge

                    // Compute child surface areas for split at edge_t
                    let other_axis0 = (axis + 1) % 3;
                    let other_axis1 = (axis + 2) % 3;
                    let below_sa = 2.0 * (d[other_axis0] * d[other_axis1] + (edge_t - node_bounds.p_min[axis]) * (d[other_axis0] + d[other_axis1]));
                    let abouve_sa = 2.0 * (d[other_axis0] * d[other_axis1] + (node_bounds.p_max[axis] - edge_t) * (d[other_axis0] + d[other_axis1]));
                    let p_below = below_sa * inv_total_sa;
                    let p_above = abouve_sa * inv_total_sa;
                    let eb = if nabove == 0 || nbelow == 0 { self.empty_bonus } else { 0.0 };
                    let cost = self.traversal_cost as Float + self.isect_cost as Float * (1.0 - eb) * (p_below * nbelow as Float + p_above * nabove as Float);

                    // Update best split if this is lowest cost so far
                    if cost < best_cost {
                        best_cost = cost;
                        best_axis = axis as isize;
                        best_offset = i as isize;
                    }
                }

                if edges[axis][i].edge_type == EdgeType::Start {
                    nbelow += 1;
                }
            }

            assert!(nbelow == n_primitives && nabove == 0);

            // Create leaf if no good splits where found
            if best_axis == -1 && retries < 2 {
                retries += 1;
                axis = (axis + 1) % 3;
                continue;
            }

            if best_cost > old_cost {
                bad_refines += 1;
            }

            if (best_cost > 4.0 * old_cost && n_primitives < 16) || best_axis == -1 || bad_refines == 3 {
                self.nodes[node_num].init_leaf(prim_nums, n_primitives, &mut self.primitive_indices);
                return
            }

            // Classify primitives with respect to split
            let mut n0 = 0;
            let mut n1 = 0;

            for i in 0..best_offset as usize {
                if edges[best_axis as usize][i].edge_type == EdgeType::Start {
                    prim0[n0] = edges[best_axis as usize][i].prim_num;
                    n0 += 1
                }
            }

            for i in (best_offset+1) as usize..(2*n_primitives) {
                if edges[best_axis as usize][i as usize].edge_type == EdgeType::End {
                    prim1[n1] = edges[best_axis as usize][i].prim_num;
                    n1 += 1;
                }
            }

            // Recursively initialize children nodes
            let tsplit = edges[best_axis as usize][best_offset as usize].t;
            let bounds0 = node_bounds;
            let bounds1 = node_bounds;

            let mut prim_nums = Vec::with_capacity(prim0.len());

            for i in 0..prim0.len() {
                prim_nums.push(prim0[i])
            }

            self.build_tree(
                node_num + 1,
                bounds0,
                all_prim_bounds,
                &mut prim_nums[..],
                n0,
                depth - 1,
                edges,
                prim0,
                &mut prim1[n_primitives..],
                bad_refines
            );

            let above_child = self.next_freenode;
            self.nodes[node_num].init_interior(best_axis as i32, above_child as i32, tsplit);

            let mut prim_nums = Vec::with_capacity(prim0.len());

            for i in 0..prim1.len() {
                prim_nums.push(prim1[i])
            }

            self.build_tree(
                above_child,
                bounds1,
                all_prim_bounds,
                &mut prim_nums[..],
                n1,
                depth - 1,
                edges,
                prim0,
                &mut prim1[n_primitives..],
                bad_refines
            );
        }
    }
}

impl Primitive for KdTreeAccel {
    fn world_bound(&self) -> Bounds3f {
        return self.bounds
    }

    fn intersect(
        &self, r: &mut Ray,
        isect: &mut SurfaceInteraction,
        _p: Arc<Primitives>) -> bool {
        let mut t_min = 0.0;
        let mut t_max = 0.0;

        if !self.bounds.intersect_p(r, &mut t_min, &mut t_max) {
            return false;
        }

        // Prepare to traverse kd-tree for ray
        let inv_dir = Vector3f::new(1.0 / r.d.x, 1.0 / r.d.y, 1.0 / r.d.z);

        let mut todo: [KdToDo; MAX_TODO] = [Default::default(); MAX_TODO];
        let mut todo_pos = 0;

        // Traverse kd-tree nodes in order for ray
        let mut hit = false;
        let mut node_idx = 0;
        let mut node = self.nodes.get(node_idx);

        while let Some(n) = node {
            // Bail out if we found a hit closer than the current node
            if r.t_max < t_min { break; }

            if !n.is_leaf() {
                // Process kd-tree interior node

                // Compute parametric distance along ray to split plane
                let axis = n.split_axis();
                let t_plane = (n.split_pos() - r.o[axis as usize]) * inv_dir[axis as usize];

                // Get node children pointers for ray
                let first_child: Option<&KdAccelNode>;
                let second_child: Option<&KdAccelNode>;
                let first_idx: usize;
                let snd_idx: usize;

                let below_first = (r.o[axis as usize] < n.split_pos()) || (r.o[axis as usize] == n.split_pos() && r.d[axis as usize] <= 0.0);

                if below_first {
                    first_idx = node_idx + 1;
                    snd_idx = n.above_child() as usize;

                } else {
                    first_idx = n.above_child() as usize;
                    snd_idx = node_idx + 1;
                }

                first_child = self.nodes.get(first_idx);
                second_child = self.nodes.get(snd_idx);

                // Advance to next child node, possibly enqueue other child
                if t_plane > t_max || t_plane < 0.0 {
                    node = first_child;
                    node_idx = first_idx;
                } else if t_plane < t_min {
                    node = second_child;
                    node_idx = snd_idx;
                } else {
                    // Enqueue second_child in todo list
                    todo[todo_pos].node = second_child;
                    todo[todo_pos].idx = snd_idx;
                    todo[todo_pos].t_min = t_plane;
                    todo[todo_pos].t_max = t_max;
                    todo_pos += 1;
                    node = first_child;
                    node_idx = first_idx;
                    t_max = t_plane;
                }
            } else {
                // Check for intersection inside leaf node
                let n_primitives = n.n_primitives();

                if n_primitives == 1 {
                    let one_prim: i32;

                    unsafe { one_prim = n.leaf.one_primitive; }

                    let p = &self.primtives[one_prim as usize];

                    // Check one primitive inside leaf node
                    if p.intersect(r, isect, p.clone()) { hit = true; }
                } else {
                    for i in 0..n_primitives {
                        let offset: i32;
                        unsafe { offset = n.leaf.primtive_offset_indices + i; }

                        let index = self.primitive_indices[offset as usize];
                        let p = &self.primtives[index];

                        // Check one primitive inside leaf node
                        if p.intersect(r, isect, p.clone()) { hit = true}
                    }
                }

                // Grab next node to process from todo list
                if todo_pos > 0 {
                    todo_pos -= 1;
                    node = todo[todo_pos].node;
                    node_idx = todo[todo_pos].idx;
                    t_min = todo[todo_pos].t_min;
                    t_max = todo[todo_pos].t_max
                } else {
                    break;
                }
            }
        }

        hit
    }

    fn intersect_p(&self, r: &mut Ray) -> bool {
        let mut t_min = 0.0;
        let mut t_max = 0.0;

        if !self.bounds.intersect_p(r, &mut t_min, &mut t_max) {
            return false;
        }

        // Prepare to traverse kd-tree for ray
        let inv_dir = Vector3f::new(1.0 / r.d.x, 1.0 / r.d.y, 1.0 / r.d.z);

        let mut todo: [KdToDo; MAX_TODO] = [Default::default(); MAX_TODO];
        let mut todo_pos = 0;

        // Traverse kd-tree nodes in order for ray
        let mut node_idx = 0;
        let mut node = self.nodes.get(node_idx);

        while let Some(n) = node {
            if n.is_leaf() {
                // Check for intersection inside leaf node
                let n_primitives = n.n_primitives();

                if n_primitives == 1 {
                    let one_prim: i32;

                    unsafe { one_prim = n.leaf.one_primitive; }

                    let p = &self.primtives[one_prim as usize];

                    // Check one primitive inside leaf node
                    if p.intersect_p(r) { return true; }
                } else {
                    for i in 0..n_primitives {
                        let offset: i32;
                        unsafe { offset = n.leaf.primtive_offset_indices + i; }

                        let index = self.primitive_indices[offset as usize];
                        let p = &self.primtives[index];

                        // Check one primitive inside leaf node
                        if p.intersect_p(r) { return true}
                    }
                }

                // Grab next node to process from todo list
                if todo_pos > 0 {
                    todo_pos -= 1;
                    node = todo[todo_pos].node;
                    node_idx = todo[todo_pos].idx;
                    t_min = todo[todo_pos].t_min;
                    t_max = todo[todo_pos].t_max
                } else {
                    break;
                }
            } else {
                // Process kd-tree interior node

                // Compute parametric distance along ray to split plane
                let axis = n.split_axis();
                let t_plane = (n.split_pos() - r.o[axis as usize]) * inv_dir[axis as usize];

                // Get node children pointers for ray
                let first_child: Option<&KdAccelNode>;
                let second_child: Option<&KdAccelNode>;
                let first_idx: usize;
                let snd_idx: usize;

                let below_first = (r.o[axis as usize] < n.split_pos()) || (r.o[axis as usize] == n.split_pos() && r.d[axis as usize] <= 0.0);

                if below_first {
                    first_idx = node_idx + 1;
                    snd_idx = n.above_child() as usize;

                } else {
                    first_idx = n.above_child() as usize;
                    snd_idx = node_idx + 1;
                }

                first_child = self.nodes.get(first_idx);
                second_child = self.nodes.get(snd_idx);

                // Advance to next child node, possibly enqueue other child
                if t_plane > t_max || t_plane < 0.0 {
                    node = first_child;
                    node_idx = first_idx;
                } else if t_plane < t_min {
                    node = second_child;
                    node_idx = snd_idx;
                } else {
                    // Enqueue second_child in todo list
                    todo[todo_pos].node = second_child;
                    todo[todo_pos].idx = snd_idx;
                    todo[todo_pos].t_min = t_plane;
                    todo[todo_pos].t_max = t_max;
                    todo_pos += 1;
                    node = first_child;
                    node_idx = first_idx;
                    t_max = t_plane;
                }
            }
        }

        false
    }

    fn get_material(&self) -> Option<Arc<Materials>> {
        panic!("BVH::get_material method called; should have gone to GeometricPrimitive")
    }

    fn get_area_light(&self) -> Option<Arc<Lights>> {
        panic!("BVH::get_area_light method called; should have gone to GeometricPrimitive")
    }

    fn compute_scattering_functions<'b: 'b>(
        &self, _isect: &mut SurfaceInteraction<'b>, _arena: &Member<'b>,
        _mode: TransportMode, _allow_multiple_lobes: bool) {
        panic!("BVH::get_compute_scattering_functions method called; should have gone to GeometricPrimitive")
    }
}

pub fn create_kdtree_accelerator(prims: Vec<Arc<Primitives>>, ps: &ParamSet) -> Arc<Primitives> {
    let isect_cost = ps.find_one_int("itersectcost", 80);
    let trav_cost = ps.find_one_int("traversalcost", 1);
    let empty_bonus = ps.find_one_float("emptybonus", 0.5);
    let max_prims = ps.find_one_int("maxprims", 1);
    let max_depth = ps.find_one_int("maxdepth", 0);
    let kd = KdTreeAccel::new(prims, isect_cost, trav_cost, empty_bonus, max_prims as usize, max_depth as usize);

    Arc::new(kd.into())
}