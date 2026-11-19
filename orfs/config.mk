export DESIGN_NAME     = ldpc_6g_decoder_top
export PLATFORM        = asap7
export VERILOG_FILES   = $(sort $(wildcard $(DESIGN_HOME)/src/$(DESIGN_NICKNAME)/*.v)) $(abspath ../rtl/ldpc_6g_decoder_top.v)
export SDC_FILE        = $(abspath constraint.sdc)

export CORE_UTILIZATION = 50
export PLACE_DENSITY    = 0.60
export TNS_END_PERCENT  = 100
