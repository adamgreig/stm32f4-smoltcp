# STM32F4 smoltcp ethernet driver

This repo is a proof of concept crate. It contains an `EthernetDevice` impl
for [smoltcp](https://github.com/m-labs/smoltcp) on an stm32f4.

For an example of usage, see [stm32f4-smoltcp-demo](https://github.com/adamgreig/stm32f4-smoltcp-demo).

To build this crate standalone, you have to enable a specific device feature of
`stm32f4xx-hal`, for example:

```
cargo build --release --features stm32f4xx-hal/rt,stm32f4xx-hal/stm32f407
```

Applications using this crate should instead enable the correct features of
`stm32f4xx-hal` themselves.
