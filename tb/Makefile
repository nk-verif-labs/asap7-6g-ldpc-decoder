SIM ?= icarus
TOPLEVEL_LANG ?= verilog

VERILOG_SOURCES += $(PWD)/../rtl/ldpc_6g_decoder_top.v

TOPLEVEL = ldpc_6g_decoder_top
MODULE = test_ldpc_6g_decoder

ifeq ($(SIM),verilator)
EXTRA_ARGS += --Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL
endif

include $(shell cocotb-config --makefiles)/Makefile.sim
