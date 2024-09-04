#![no_std]
#![feature(linkage)]
#![allow(dead_code)]
#![allow(unused)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

extern crate alloc;

mod mii_const;
mod macb_const;

mod fxmac_phy;
mod fxmac_dma;
pub mod fxmac;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
