# Tensor-conrtaction

This repository contains a SystemVerilog processing element (`pe.sv`) for a
tensor-contraction accelerator. The design supports streaming, broadcast, and
trace modes with 4-lane SIMD datapaths, pipelined floating-point multipliers
and adders, and optional accumulator clearing via the `Clear_acc` control
signal.
