use lazy_static::lazy_static;
use state::{Storage, LocalStorage};
use std::sync::Mutex;
use combine::lib::collections::HashMap;

type StatsCallbackFn = Box<dyn Fn(&mut StatsAccumulator) + Send>;

lazy_static! {
    static ref FUNCS: Storage<Mutex<Vec<StatsCallbackFn>>> = Storage::new();
    static ref STATS_ACCUMULATOR: Storage<Mutex<StatsAccumulator>> = Storage::new();
}

#[macro_export]
macro_rules! stat_counter {
    ($title:expr, $f:ident) => {

        mod $f {
            use crate::core::stats::{StatsRegisterer, StatsAccumulator};
            use state::LocalStorage;
            use std::cell::Cell;
            use lazy_static::lazy_static;

            lazy_static! {
                static ref VALUE: LocalStorage<Cell<u64>> = LocalStorage::new();
            }

            pub fn init() {
                VALUE.set(|| Cell::new(0));
                StatsRegisterer::register(report);
            }

            pub fn inc() {
                let v = VALUE.get();
                v.set(v.get() + 1);
            }

            fn report(accum: &mut StatsAccumulator) {
                accum.report_counter($title, VALUE.get().get());
                VALUE.get().set(0);
            }
        }

    }
}

#[macro_export]
macro_rules! stat_memory_counter {
    ($title:expr, $f:ident) => {

        mod $f {
            use crate::core::stats::{StatsRegisterer, StatsAccumulator};
            use state::LocalStorage;
            use std::cell::Cell;
            use lazy_static::lazy_static;

            lazy_static! {
                static ref VALUE: LocalStorage<Cell<u64>> = LocalStorage::new();
            }

            pub fn init() {
                VALUE.set(|| Cell::new(0));
                StatsRegisterer::register(report);
            }

            pub fn add(val: u64) {
                let v = VALUE.get();
                v.set(v.get() + val);
            }

            fn report(accum: &mut StatsAccumulator) {
                accum.report_memory_counter($title, VALUE.get().get());
                VALUE.get().set(0);
            }
        }
    }
}

#[macro_export]
macro_rules! stat_int_distribution {
    ($title:expr, $f:ident) => {
        mod $f {
            use crate::core::stats::{StatsRegisterer, StatsAccumulator};
            use state::LocalStorage;
            use std::cell::Cell;
            use lazy_static::lazy_static;

            lazy_static! {
                static ref SUM: LocalStorage<Cell<u64>> = LocalStorage::new();
                static ref COUNT: LocalStorage<Cell<u64>> = LocalStorage::new();
                static ref MIN: LocalStorage<Cell<u64>> = LocalStorage::new();
                static ref MAX: LocalStorage<Cell<u64>> = LocalStorage::new();
            }

            pub fn init() {
                SUM.set(|| Cell::new(0));
                COUNT.set(|| Cell::new(0));
                MIN.set(|| Cell::new(u64::MAX));
                MAX.set(|| Cell::new(u64::MIN));
                StatsRegisterer::register(report);
            }

            fn report(accum: &mut StatsAccumulator) {
                accum.report_int_distribution(
                    $title,
                    SUM.get().get(),
                    COUNT.get().get(),
                    MIN.get().get(),
                    MAX.get().get()
                );

                SUM.get().set(0);
                COUNT.get().set(0);
                MIN.get().set(u64::MAX);
                MAX.get().set(u64::MIN);
            }

            pub fn report_value(value: u64) {
                let sum = SUM.get();
                sum.set(sum.get() + value);
                let count = COUNT.get();
                count.set(count.get() + 1);
                let min = MIN.get();
                min.set(u64::min(min.get(), value));
                let max = MAX.get();
                max.set(u64::max(max.get(), value));
            }
        }
    }
}

#[macro_export]
macro_rules! stat_float_distribution {
    ($title:expr, $f:ident) => {
        mod $f {
            use crate::core::stats::{StatsRegisterer, StatsAccumulator};
            use state::LocalStorage;
            use std::cell::Cell;
            use lazy_static::lazy_static;

            lazy_static! {
                static ref SUM: LocalStorage<Cell<f64>> = LocalStorage::new();
                static ref COUNT: LocalStorage<Cell<u64>> = LocalStorage::new();
                static ref MIN: LocalStorage<Cell<f64>> = LocalStorage::new();
                static ref MAX: LocalStorage<Cell<f64>> = LocalStorage::new();
            }

            pub fn init() {
                SUM.set(|| Cell::new(0.0));
                COUNT.set(|| Cell::new(0));
                MIN.set(|| Cell::new(f64::MAX));
                MAX.set(|| Cell::new(f64::MIN));
                StatsRegisterer::register(report);
            }

            fn report(accum: &mut StatsAccumulator) {
                accum.report_float_distribution(
                    $title,
                    SUM.get().get(),
                    COUNT.get().get(),
                    MIN.get().get(),
                    MAX.get().get()
                );

                SUM.get().set(0.0);
                COUNT.get().set(0);
                MIN.get().set(f64::MAX);
                MAX.get().set(f64::MIN);
            }

            pub fn report_value(value: f64) {
                let sum = SUM.get();
                sum.set(sum.get() + value);
                let count = COUNT.get();
                count.set(count.get() + 1);
                let min = MIN.get();
                min.set(f64::min(min.get(), value));
                let max = MAX.get();
                max.set(f64::max(max.get(), value));
            }
        }
    }
}

#[macro_export]
macro_rules! stat_percent {
    ($title:expr, $f:ident) => {

        mod $f {
            use crate::core::stats::{StatsRegisterer, StatsAccumulator};
            use state::LocalStorage;
            use std::cell::Cell;
            use lazy_static::lazy_static;

            lazy_static! {
                static ref NUM: LocalStorage<Cell<u64>> = LocalStorage::new();
                static ref DENOM: LocalStorage<Cell<u64>> = LocalStorage::new();
            }

            pub fn inc_num() {
                let v = NUM.get();
                v.set(v.get() + 1);
            }

            pub fn inc_den() {
                let v = DENOM.get();
                v.set(v.get() + 1);
            }

            pub fn init() {
                NUM.set(|| Cell::new(0));
                DENOM.set(|| Cell::new(0));
                StatsRegisterer::register(report);
            }

            fn report(accum: &mut StatsAccumulator) {
                accum.report_percentage($title, NUM.get().get(), DENOM.get().get());
                NUM.get().set(0);
            }
        }
    }
}

#[macro_export]
macro_rules! stat_ratio {
    ($title:expr, $f:ident) => {

        mod $f {
            use crate::core::stats::{StatsRegisterer, StatsAccumulator};
            use state::LocalStorage;
            use std::cell::Cell;
            use lazy_static::lazy_static;

            lazy_static! {
                static ref NUM: LocalStorage<Cell<u64>> = LocalStorage::new();
                static ref DENOM: LocalStorage<Cell<u64>> = LocalStorage::new();
            }

            pub fn init() {
                NUM.set(|| Cell::new(0));
                DENOM.set(|| Cell::new(0));
                StatsRegisterer::register(report);
            }

            pub fn inc_num() {
                let v = NUM.get();
                v.set(v.get() + 1);
            }

            pub fn inc_den() {
                let v = DENOM.get();
                v.set(v.get() + 1);
            }

            fn report(accum: &mut StatsAccumulator) {
                accum.report_ratio($title, NUM.get().get(), DENOM.get().get());
                NUM.get().set(0);
            }
        }
    }
}

pub struct StatsRegisterer();

impl StatsRegisterer {
    pub fn register<F: 'static +  Fn(&mut StatsAccumulator) + Send>(func: F)  {
        let mut funcs = FUNCS.get().lock().unwrap();

        funcs.push(Box::new(func));
    }

    pub fn call_callbacks(accum: &mut StatsAccumulator) {
        let funcs = FUNCS.get().lock().unwrap();

        for func in funcs.iter() {
            func(accum)
        }
    }
}

#[derive(Default)]
pub struct StatsAccumulator {
    counters                    : HashMap<String, u64>,
    memory_counters             : HashMap<String, u64>,
    int_distribution_sums       : HashMap<String, u64>,
    int_distribution_counts     : HashMap<String, u64>,
    int_distribution_mins       : HashMap<String, u64>,
    int_distribution_maxs       : HashMap<String, u64>,
    float_distribution_sums     : HashMap<String, f64>,
    float_distribution_counts   : HashMap<String, u64>,
    float_distribution_mins     : HashMap<String, f64>,
    float_distribution_maxs     : HashMap<String, f64>,
    percentages                 : HashMap<String, (u64, u64)>,
    ratios                      : HashMap<String, (u64, u64)>
}

impl StatsAccumulator {
    pub fn new() -> Self {
        Default::default()
    }
    
    pub fn report_counter(&mut self, name: &str, val: u64) {
        let c = self.counters.entry(name.to_owned()).or_insert(0);

        *c += val;
    }

    pub fn report_memory_counter(&mut self, name: &str, val: u64) {
        let c = self.memory_counters.entry(name.to_owned()).or_insert(0);

        *c += val;
    }

    pub fn report_int_distribution(&mut self, name: &str, sum: u64, count: u64, min: u64, max: u64) {
        let s = self.int_distribution_sums.entry(name.to_owned()).or_insert(0);
        *s += sum;

        let c = self.int_distribution_counts.entry(name.to_owned()).or_insert(0);
        *c += count;

        let mn = self.int_distribution_mins.entry(name.to_owned()).or_insert(min);
        *mn = u64::min(*mn, min);

        let mx = self.int_distribution_maxs.entry(name.to_owned()).or_insert(max);
        *mx = u64::min(*mx, max);
    }

    pub fn report_float_distributions(&mut self, name: &str, sum: f64, count: u64, min: f64, max: f64) {
        let s = self.float_distribution_sums.entry(name.to_owned()).or_insert(0.0);
        *s += sum;

        let c = self.float_distribution_counts.entry(name.to_owned()).or_insert(0);
        *c += count;

        let mn = self.float_distribution_mins.entry(name.to_owned()).or_insert(min);
        *mn = f64::min(*mn, min);

        let mx = self.float_distribution_maxs.entry(name.to_owned()).or_insert(max);
        *mx = f64::min(*mx, max);
    }

    pub fn report_percentage(&mut self, name: &str, num: u64, denom: u64) {
        let perc = self.percentages.entry(name.to_owned()).or_insert((0, 0));

        *perc = (perc.0 + num, perc.1 + denom);
    }

    pub fn report_ratio(&mut self, name: &str, num: u64, denom: u64) {
        let r = self.ratios.entry(name.to_owned()).or_insert((0, 0));

        *r = (r.0 + num, r.1 + denom);
    }

    pub fn clear(&mut self) {
        self.counters.clear();
        self.memory_counters.clear();
        self.int_distribution_mins.clear();
        self.int_distribution_sums.clear();
        self.int_distribution_counts.clear();
        self.int_distribution_maxs.clear();
        self.float_distribution_counts.clear();
        self.float_distribution_maxs.clear();
        self.float_distribution_mins.clear();
        self.float_distribution_sums.clear();
        self.percentages.clear();
        self.ratios.clear();
    }
    
    pub fn print<W: std::io::Write>(&self, writer: W) {
        // TODO: print()
        unimplemented!()
    } 
}

pub fn init_stats() {
    STATS_ACCUMULATOR.set(Mutex::new(Default::default()));
    FUNCS.set(Mutex::new(Vec::new()));
}

pub fn print_stats<W: std::io::Write>(writer: W) {
    let acc = STATS_ACCUMULATOR.get().lock().unwrap();
    acc.print(writer);
}

pub fn report_stats() {
    let acc = &mut STATS_ACCUMULATOR.get().lock().unwrap();

    StatsRegisterer::call_callbacks(acc);
}