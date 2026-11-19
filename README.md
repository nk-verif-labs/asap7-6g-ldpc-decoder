# asap7-6g-ldpc-decoder

**1.0 GHz `(12, 6)` QC-LDPC Normalized Min-Sum Decoder** targeting **ASAP7 (7 nm FinFET predictive PDK)**.

## Architecture Overview
- **Top Module:** `ldpc_6g_decoder_top`
- **Target Clock:** `1.0 GHz` (`1000 ps` in ASAP7 time units)
- **Algorithm:** Iterative Normalized Min-Sum ($\alpha = 0.75$, implemented shift-add `(m >> 1) + (m >> 2)`) over 6-bit signed LLRs (`Q4.2`) with early zero-syndrome termination.
- **Interfaces:**
  - `s_axil` (AXI4-Lite): `0x00 CTRL` (`max_iter`), `0x04 STATUS`, `0x08 DECODE_CNT`, `0x0C SYND_ZERO`
  - `s_axis` (`72-bit`): 12 × 6-bit signed channel LLRs
  - `m_axis` (`12-bit`): Hard-decision decoded codeword

## Quick Start
```bash
source ~/.venvs/cocotb19/bin/activate
make -C tb SIM=icarus
make -C tb SIM=verilator
```
