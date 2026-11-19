import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

P = [
    [1, 1, 1, 0, 0, 0],
    [0, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 0],
    [0, 0, 0, 1, 1, 1],
    [1, 0, 0, 0, 1, 1],
    [1, 1, 0, 0, 0, 1],
]


def encode_cw(msg_int: int) -> int:
    msg = [(msg_int >> i) & 1 for i in range(6)]
    cw = list(msg)
    for r in range(6):
        cw.append(sum(P[r][c] * msg[c] for c in range(6)) & 1)
    out = 0
    for i, b in enumerate(cw):
        out |= (b << i)
    return out


def pack_llrs(llrs: list[int]) -> int:
    val = 0
    for i, l in enumerate(llrs):
        val |= ((l & 0x3F) << (i * 6))
    return val


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.s_axil_awaddr.value = 0
    dut.s_axil_awvalid.value = 0
    dut.s_axil_wdata.value = 0
    dut.s_axil_wstrb.value = 0
    dut.s_axil_wvalid.value = 0
    dut.s_axil_bready.value = 1
    dut.s_axil_araddr.value = 0
    dut.s_axil_arvalid.value = 0
    dut.s_axil_rready.value = 1
    dut.s_axis_tdata.value = 0
    dut.s_axis_tvalid.value = 0
    dut.m_axis_tready.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def axil_read(dut, addr: int) -> int:
    dut.s_axil_araddr.value = addr
    dut.s_axil_arvalid.value = 1
    val = 0
    for _ in range(20):
        await RisingEdge(dut.clk)
        if dut.s_axil_arready.value:
            dut.s_axil_arvalid.value = 0
        if dut.s_axil_rvalid.value:
            val = int(dut.s_axil_rdata.value)
            break
    await RisingEdge(dut.clk)
    return val


async def decode_one(dut, llrs: list[int]) -> int:
    dut.s_axis_tdata.value = pack_llrs(llrs)
    dut.s_axis_tvalid.value = 1
    while True:
        await RisingEdge(dut.clk)
        if dut.s_axis_tready.value:
            break
    dut.s_axis_tvalid.value = 0

    for _ in range(40):
        await RisingEdge(dut.clk)
        if dut.m_axis_tvalid.value:
            return int(dut.m_axis_tdata.value)
    raise AssertionError("Timed out waiting for LDPC decoder output")


@cocotb.test()
async def test_ldpc_error_free_and_single_bit_correction(dut):
    """Verify early syndrome pass on clean codewords and 1-bit error correction."""
    cocotb.start_soon(Clock(dut.clk, 1, units="ns").start())  # 1.0 GHz target
    await reset_dut(dut)

    # 1. Clean codewords (should decode with 0 iterations)
    for msg in [0x00, 0x15, 0x2A, 0x3F]:
        cw = encode_cw(msg)
        llrs = [12 if ((cw >> i) & 1) == 0 else -12 for i in range(12)]
        dec = await decode_one(dut, llrs)
        assert dec == cw, f"Clean decode mismatch: got {hex(dec)}, exp {hex(cw)}"

    # 2. Inject 1-bit channel error on every bit position
    for flip in range(12):
        msg = random.randint(0, 63)
        cw = encode_cw(msg)
        llrs = [12 if ((cw >> i) & 1) == 0 else -12 for i in range(12)]
        llrs[flip] = -llrs[flip]
        dec = await decode_one(dut, llrs)
        assert (dec & 0x3F) == msg, f"1-bit error on bit {flip} failed: got {hex(dec & 0x3F)}, exp {hex(msg)}"

    decode_cnt = await axil_read(dut, 0x08)
    synd_zero = await axil_read(dut, 0x0C)
    assert decode_cnt == 16, f"Expected DECODE_CNT=16, got {decode_cnt}"
    assert synd_zero == 16, f"Expected all 16 frames to reach zero syndrome, got {synd_zero}"
