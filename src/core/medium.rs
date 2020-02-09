use std::fmt::Debug;
use std::sync::Arc;

pub trait Medium: Debug{}

#[derive(Debug, Clone)]
pub struct MediumInterface{
    pub inside: Arc<dyn Medium>,
    pub outside: Arc<dyn Medium>
}