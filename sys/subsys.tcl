set scrDir [file dirname [file normalize [info script]]]

read_verilog -sv [file join $scrDir sys_top.sv ]
read_verilog -sv [file join $scrDir subsys.sv ]
read_vhdl        [file join $scrDir ascal.vhd ]
read_vhdl        [file join $scrDir pll_hdmi_adj.vhd ]
read_verilog -sv [file join $scrDir hq2x.sv ]
read_verilog     [file join $scrDir scandoubler.v ]
read_verilog     [file join $scrDir scanlines.v ]
read_verilog -sv [file join $scrDir video_cleaner.sv ]
read_verilog -sv [file join $scrDir gamma_corr.sv ]
read_verilog -sv [file join $scrDir video_mixer.sv ]
read_verilog     [file join $scrDir arcade_video.v ]
read_verilog     [file join $scrDir osd.v ]
read_verilog -sv [file join $scrDir vga_out.sv ]
read_verilog     [file join $scrDir i2c.v ]
read_verilog -sv [file join $scrDir alsa.sv ]
read_verilog     [file join $scrDir i2s.v ]
read_verilog     [file join $scrDir spdif.v ]
read_verilog     [file join $scrDir audio_out.v ]
read_verilog -sv [file join $scrDir ltc2308.sv ]
read_verilog     [file join $scrDir sigma_delta_dac.v ]
read_verilog -sv [file join $scrDir hdmi_config.sv ]
read_verilog -sv [file join $scrDir mcp23009.sv ]
read_verilog -sv [file join $scrDir ddr_svc.sv ]
read_verilog -sv [file join $scrDir sysmem.sv ]
read_verilog     [file join $scrDir sd_card.v ]
read_verilog     [file join $scrDir hps_io.v ]
