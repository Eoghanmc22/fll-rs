#!/bin/bash

cargo build --release --example minimal
scp ./target/armv5te-unknown-linux-musleabi/release/examples/minimal robot@ev3dev.local:/home/robot/minimal
ssh robot@ev3dev.local "RUST_BACKTRACE=full brickrun ~/minimal"
