use crate::core::pbrt::Float;
use anyhow::{Result, Context};
use std::path::Path;
use std::fs::File;
use::std::io::{BufRead, BufReader};
use log::warn;


pub fn read_float_file(name: &str, values: &mut Vec<Float>) -> Result<()> {
    let path = Path::new(name);
    let res = File::open(path).with_context(|| format!("Error: failed to open file {}", name))?;
    let reader = BufReader::new(res);

    for (n, l) in reader.lines().enumerate() {
        let line = l?;

        if !line.is_empty() {
            if line.starts_with('#') {
                continue;
            }

            for token in line.split_whitespace() {
                match token.parse::<Float>() {
                    Ok(val) => values.push(val),
                    Err(_) => warn!("Unexpected text found at line {} of float file {}", n, name)
                }

                let val = token.parse::<Float>()?;
                values.push(val);
            }

        }
    }

    Ok(())
}