pub trait Shape{
    fn reverse_orientation(&self) -> bool;
    fn transform_swapshandedness(&self) -> bool;
}