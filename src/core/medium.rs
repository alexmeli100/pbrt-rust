use std::fmt::Debug;
use std::sync::Arc;

#[derive(Debug)]
pub enum Medium {

}

#[derive(Debug, Clone)]
pub struct MediumInterface{
    pub inside: Arc<Medium>,
    pub outside: Arc<Medium>
}