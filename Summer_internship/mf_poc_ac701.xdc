# ==============================================================================
# mf_poc_ac701.xdc  -  AC701 (XC7A200T-2FBG676)
# Target top module : mf_poc_top   (matched-filter PROOF OF CONCEPT, UART in/out)
#
# Same board pinout as mf_bf_uart_ac701.xdc (identical ports: sys_clk_p/n,
# reset=SW7, uart_rxd, uart_txd, led[3:0]). Only difference from that file:
#   * the clock-divider register in mf_poc_top.v is named `div` (2-bit), so the
#     generated clock is sourced from div_reg[1]/Q (was clk_div_reg[1]/Q).
#   * LED meanings updated for the MF PoC FSM.
# 200/4 = 50 MHz core, CLKS_PER_BIT = 434 (115200 baud) in the RTL.
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. SYSTEM CLOCK (200 MHz differential, bank 34)
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN  R3          [get_ports sys_clk_p]
set_property IOSTANDARD   DIFF_SSTL15 [get_ports sys_clk_p]
set_property PACKAGE_PIN  P3          [get_ports sys_clk_n]
set_property IOSTANDARD   DIFF_SSTL15 [get_ports sys_clk_n]
set_property INTERNAL_VREF 0.75       [get_iobanks 34]
create_clock -period 5.000 -name sys_clk_200 -waveform {0.000 2.500} [get_ports sys_clk_p]

# ------------------------------------------------------------------------------
# 2. CORE CLOCK = 200/4 = 50 MHz  (FF divider: div[1] -> BUFG -> clk)
#    mf_poc_top.v declares `reg [1:0] div;`, so its bit-1 flop is div_reg[1].
#    If you change the divide ratio in the RTL, update -divide_by here to match.
# ------------------------------------------------------------------------------
create_generated_clock -name clk_core -source [get_ports sys_clk_p] \
    -divide_by 4 [get_pins {div_reg[1]/Q}]

# ------------------------------------------------------------------------------
# 3. SYSTEM RESET (AC701 SW7, active high)
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN U4     [get_ports reset]
set_property IOSTANDARD  SSTL15 [get_ports reset]
# reset is a control signal, not a clock - suppress the illegal BUFG attempt:
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets reset_IBUF]

# ------------------------------------------------------------------------------
# 4. UART (USB-UART bridge, FPGA U1 <-> CP2103 U44, LVCMOS18 per UG952 Table 1-18)
#    Pin names in UG952 are from the BRIDGE's view, so they look reversed:
#      T19 = "USB_UART_TX" = bridge transmit  = data INTO  fpga = uart_rxd
#      U19 = "USB_UART_RX" = bridge receive   = data OUT of fpga = uart_txd
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN U19      [get_ports uart_txd]
set_property IOSTANDARD  LVCMOS18 [get_ports uart_txd]

set_property PACKAGE_PIN T19      [get_ports uart_rxd]
set_property IOSTANDARD  LVCMOS18 [get_ports uart_rxd]

# uart_rxd is asynchronous; uart_rx has a 2-FF synchronizer, so the input path is
# a false path (slow 115200 baud, exact pin timing irrelevant).
set_false_path -from [get_ports uart_rxd]

# ------------------------------------------------------------------------------
# 5. DIAGNOSTIC LEDS  (mf_poc_top FSM)
#    0 = heartbeat (free-running)     1 = filtering (S_FEED/S_DRAIN)
#    2 = streaming out (S_TX_PRE+)    3 = complete (S_DONE)
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN M26 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN T24 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN T25 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN R26 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]

# ------------------------------------------------------------------------------
# 6. OUTPUT FALSE PATHS (single 50 MHz clock domain -> no async groups needed)
# ------------------------------------------------------------------------------
set_false_path -to [get_ports uart_txd]
set_false_path -to [get_ports {led[*]}]

# ------------------------------------------------------------------------------
# 7. BITSTREAM CONFIGURATION
# ------------------------------------------------------------------------------
set_property CFGBVS         VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3  [current_design]
