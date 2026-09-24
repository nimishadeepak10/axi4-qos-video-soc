# synth.tcl - non-project batch-mode synthesis + full place & route of
# axi4_qos_videosoc_top. Run with: vivado -mode batch -source vivado/synth.tcl
# Produces utilization and timing reports under vivado/reports/.
#
# Only the synthesizable RTL is read here - ddr_behavioral_model.sv is
# simulation-only (see its header comment) and is deliberately excluded,
# same as no real design synthesizes its external DRAM.

set proj_dir   [file normalize [file dirname [info script]]]
set rtl_dir    [file normalize "$proj_dir/../rtl"]
set report_dir "$proj_dir/reports"
file mkdir $report_dir

set_param general.maxThreads 1

read_verilog -sv [list \
    $rtl_dir/axi4_write_arb.sv \
    $rtl_dir/axi4_read_arb.sv \
    $rtl_dir/axi4_dma_2d.sv \
    $rtl_dir/fifo_sync.sv \
    $rtl_dir/axi4_ddr_ctrl.sv \
    $rtl_dir/frame_buffer_mgr.sv \
    $rtl_dir/axi4_qos_videosoc_top.sv \
]

set_property top axi4_qos_videosoc_top [current_fileset]

# Target device note: this Vivado install has only the 7-series Artix
# device support package installed (no Zynq-7000 parts available -
# `get_parts -filter {NAME =~ "*7z020*"}` returns empty here). Zynq-7020's
# PL is built from the identical 7-series logic fabric as Artix-7 (same
# slice/LUT/FF/BRAM/CARRY4 primitives, same speed-grade characterization
# family), so xc7a100t is used as a same-fabric stand-in and the results
# are directly comparable - documented here rather than silently
# substituted.
set part xc7a100tcsg324-1

synth_design -top axi4_qos_videosoc_top -part $part -mode out_of_context

create_clock -name clk -period 5.000 [get_ports clk]

opt_design
place_design
phys_opt_design
route_design

report_utilization -file $report_dir/utilization.rpt
report_timing_summary -file $report_dir/timing_summary.rpt
report_timing -delay_type max -max_paths 5 -file $report_dir/timing_worst_paths.rpt

write_checkpoint -force $report_dir/post_route.dcp

puts "SYNTH_DONE"
