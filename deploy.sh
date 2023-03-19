#!/bin/bash

cargo build --release --example simple
scp ./target/armv5te-unknown-linux-musleabi/release/examples/simple robot@ev3dev.local:/home/robot/simple
ssh robot@ev3dev.local "RUST_BACKTRACE=full brickrun ~/simple"
