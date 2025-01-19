#![no_std]
#![feature(linkage)]
#![allow(dead_code)]
#![allow(unused)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

extern crate alloc;

#[macro_use]
extern crate log;

//mod mii_const;
mod fxmac_const;

mod utils;
mod fxmac_phy;
mod fxmac_dma;
mod fxmac_intr;
pub mod fxmac;

pub use fxmac::{FXmac, xmac_init};
pub use fxmac_dma::{FXmacLwipPortTx, FXmacRecvHandler};

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
}
