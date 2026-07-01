# ==============================================================================
# mf_bf_uart_ac701.xdc  -  AC701 (XC7A200T-2FBG676)
# Target top module : mf_bf_uart_top   (NO Ethernet; UART in/out only)
#
# Changes vs the old eth+uart XDC:
#   * removed all RGMII/PHY pins, the 125 MHz phy clock, and the async groups
#   * removed the malformed create_generated_clock (had stray -multiply_by lines)
#   * core clock is now 200/4 = 50 MHz (matches your datapath closure; 100 fails)
#   * ADDED uart_rxd (host -> FPGA) for the START command
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
# 2. CORE CLOCK = 200/4 = 50 MHz  (FF divider: clk_div[1] -> BUFG -> clk)
#    50 MHz is the frequency your W2/W3 datapath closes at. The divider reg in
#    mf_bf_uart_top.v is `reg [1:0] clk_div;`, so its bit-1 flop is clk_div_reg[1].
#    If you change the divide ratio in the RTL, update -divide_by here to match.
# ------------------------------------------------------------------------------
create_generated_clock -name clk_core -source [get_ports sys_clk_p] \
    -divide_by 4 [get_pins {clk_div_reg[1]/Q}]

# ------------------------------------------------------------------------------
# 3. SYSTEM RESET (AC701 SW7, active high)
# ------------------------------------------------------------------------------

set_property PACKAGE_PIN U4     [get_ports reset]
set_property IOSTANDARD  SSTL15 [get_ports reset]
# reset is a control signal, not a clock - suppress the illegal BUFG attempt:
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets reset_IBUF]

# 4. UART (USB-UART bridge, FPGA U1 <-> CP2103 U44, LVCMOS18 per UG952 Table 1-18)
#    Pin names in UG952 are from the BRIDGE's view, so they look reversed:
#      T19 = "USB_UART_TX" = bridge transmit  = data INTO  fpga = uart_rxd
#      U19 = "USB_UART_RX" = bridge receive   = data OUT of fpga = uart_txd
set_property PACKAGE_PIN U19      [get_ports uart_txd]
set_property IOSTANDARD  LVCMOS18 [get_ports uart_txd]

set_property PACKAGE_PIN T19      [get_ports uart_rxd]
set_property IOSTANDARD  LVCMOS18 [get_ports uart_rxd]

set_false_path -from [get_ports uart_rxd]

# uart_rxd is asynchronous; the uart_rx module has a 2-FF synchronizer, so the
# input path is a false path (slow 115200 baud, exact pin timing is irrelevant).
set_false_path -from [get_ports uart_rxd]

# ------------------------------------------------------------------------------
# 5. DIAGNOSTIC LEDS
#    0 = heartbeat (free-running)   1 = pipeline running / streaming
#    2 = pipeline done (sticky)     3 = stream complete (sticky)
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
