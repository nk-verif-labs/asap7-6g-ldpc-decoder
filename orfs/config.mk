export DESIGN_NAME      = ldpc_6g_decoder_top
export PLATFORM         = asap7
ORFS_CFG_DIR            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
export VERILOG_FILES    = $(abspath $(ORFS_CFG_DIR)/../rtl/ldpc_6g_decoder_top.v)
export SDC_FILE         = $(abspath $(ORFS_CFG_DIR)/constraint.sdc)
export CORE_UTILIZATION = 50
export PLACE_DENSITY    = 0.60
export TNS_END_PERCENT  = 100
