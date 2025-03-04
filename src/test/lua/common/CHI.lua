local utils = require "LuaUtils"
local setmetatable = setmetatable
local lshift = bit.lshift
local enum_search = utils.enum_search

local OpcodeREQ = setmetatable({
	name = "OpcodeREQ",

	ReqLCrdReturn = 0x00 + lshift(0, 6),
	ReadShared = 0x01 + lshift(0, 6),
	MakeReadUnique = 0x01 + lshift(1, 6),
	ReadClean = 0x02 + lshift(0, 6),
	WriteEvictOrEvict = 0x02 + lshift(1, 6),
	ReadOnce = 0x03 + lshift(0, 6),
	WriteUniqueZero = 0x03 + lshift(1, 6),
	ReadNoSnp = 0x04 + lshift(0, 6),
	PCrdReturn = 0x05 + lshift(0, 6),
	ReadUnique = 0x07 + lshift(0, 6),
	CleanShared = 0x08 + lshift(0, 6),
	CleanInvalid = 0x09 + lshift(0, 6),
	MakeInvalid = 0x0a + lshift(0, 6),
	CleanUnique = 0x0b + lshift(0, 6),
	MakeUnique = 0x0c + lshift(0, 6),
	Evict = 0x0d + lshift(0, 6),

	WriteEvictFull = 0x15 + lshift(0, 6),
	WriteCleanFull = 0x17 + lshift(0, 6),
	WriteUniquePtl = 0x18 + lshift(0, 6),
	WriteUniqueFull = 0x19 + lshift(0, 6),
	WriteBackPtl = 0x1A + lshift(0, 6),
	WriteBackFull = 0x1B + lshift(0, 6),
	WriteNoSnpPtl = 0x1C + lshift(0, 6),
	WriteNoSnpFull = 0x1D + lshift(0, 6),
	ReadOnceCleanInvalid = 0x24 + lshift(0, 6),
	ReadOnceMakeInvalid = 0x25 + lshift(0, 6),

	ReadNotSharedDirty = 0x26 + lshift(0, 6),

	AtomicStore_ADD = 0x28 + lshift(0, 6),
	AtomicStore_CLR = 0x29 + lshift(0, 6),
	AtomicStore_EOR = 0x2A + lshift(0, 6),
	AtomicStore_SET = 0x2B + lshift(0, 6),
	AtomicStore_SMAX = 0x2C + lshift(0, 6),
	AtomicStore_SMIN = 0x2D + lshift(0, 6),
	AtomicStore_UMAX = 0x2E + lshift(0, 6),
	AtomicStore_UMIN = 0x2F + lshift(0, 6),

	AtomicLoad_ADD = 0x30 + lshift(0, 6),
	AtomicLoad_CLR = 0x31 + lshift(0, 6),
	AtomicLoad_EOR = 0x32 + lshift(0, 6),
	AtomicLoad_SET = 0x33 + lshift(0, 6),
	AtomicLoad_SMAX = 0x34 + lshift(0, 6),
	AtomicLoad_SMIN = 0x35 + lshift(0, 6),
	AtomicLoad_UMAX = 0x36 + lshift(0, 6),
	AtomicLoad_UMIN = 0x37 + lshift(0, 6),

	AtomicSwap = 0x38 + lshift(0, 6),
	AtomicCompare = 0x39 + lshift(0, 6),
}, { __call = enum_search })

local OpcodeDAT = setmetatable({
	name = "OpcodeDAT",

	DataLCrdReturn = 0x00,
	SnpRespData = 0x01,
	CopyBackWrData = 0x02,
	NonCopyBackWrData = 0x03,
	CompData = 0x04,
	SnpRespDataPtl = 0x05,
	SnpRespDataFwded = 0x06,
	WriteDataCancel = 0x07,
	DataSepResp = 0x0B,
	NCBWrDataCompAck = 0x0C,
}, { __call = enum_search })

local OpcodeRSP = setmetatable({
	name = "OpcodeRSP",

	RespLCrdReturn = 0x00,
	SnpResp = 0x01,
	CompAck = 0x02,
	RetryAck = 0x03,
	Comp = 0x04,
	CompDBIDResp = 0x05,
	DBIDResp = 0x06,
	PCrdGrant = 0x07,
	ReadReceipt = 0x08,
}, { __call = enum_search })

local OpcodeSNP = setmetatable({
	name = "OpcodeSNP",

	SnpLCrdReturn = 0x00,
	SnpShared = 0x01,
	SnpClean = 0x02,
	SnpOnce = 0x03,
	SnpNotSharedDirty = 0x04,
	SnpUnique = 0x07,
	SnpCleanShared = 0x08,
	SnpCleanInvalid = 0x09,
	SnpMakeInvalid = 0x0A,
	SnpQuery = 0x10,
	SnpSharedFwd = 0x11,
	SnpCleanFwd = 0x12,
	SnpOnceFwd = 0x13,
	SnpNotSharedDirtyFwd = 0x14,
	SnpPreferUnique = 0x15,
	SnpPreferUniqueFwd = 0x16,
	SnpUniqueFwd = 0x17,
}, { __call = enum_search })

local function dat_has_data(opcode)
	return opcode == OpcodeDAT.SnpRespData or opcode == OpcodeDAT.CopyBackWrData or opcode == OpcodeDAT.CompData or opcode == OpcodeDAT.NonCopyBackWrData
end

local function dat_is_snpresp(opcode)
	return opcode == OpcodeDAT.SnpRespData or opcode == OpcodeDAT.SnpRespDataPtl or opcode == OpcodeDAT.SnpRespDataFwded
end

local function req_need_data(opcode)
	return opcode == OpcodeREQ.ReadShared or opcode == OpcodeREQ.ReadUnique or opcode == OpcodeREQ.ReadNotSharedDirty
end

local function is_fwd_snoop(opcode)
	return opcode == OpcodeSNP.SnpSharedFwd or opcode == OpcodeSNP.SnpUniqueFwd or opcode == OpcodeSNP.SnpCleanFwd or opcode == OpcodeSNP.SnpOnceFwd or opcode == OpcodeSNP.SnpNotSharedDirtyFwd
end

local CHIResp = utils.enum_define({
	name = "CHIResp",

	I = ("0b000"):number(),
	SC = ("0b001"):number(),
	UC = ("0b010"):number(),
	UD = ("0b010"):number(), -- for Snoop responses
	SD = ("0b011"):number(), -- for Snoop responses
	I_PD = ("0b100"):number(), -- for Snoop responses
	SC_PD = ("0b101"):number(), -- for Snoop responses
	UC_PD = ("0b110"):number(), -- for Snoop responses
	UD_PD = ("0b110"):number(),
	SD_PD = ("0b111"):number(),
})

local CHIOrder = utils.enum_define({
	name = "CHIOrder",

	None = ("0b00"):number(),
	RequestAccepted = ("0b01"):number(),
	RequestOrder = ("0b10"):number(),
	OWO = ("0b10"):number(), -- Ordered Write Observation
	EndpointOrder = ("0b11"):number(),
})

return {
	OpcodeREQ = OpcodeREQ,
	OpcodeDAT = OpcodeDAT,
	OpcodeRSP = OpcodeRSP,
	OpcodeSNP = OpcodeSNP,
	CHIResp = CHIResp,
	CHIOrder = CHIOrder,
	dat_has_data = dat_has_data,
	req_need_data = req_need_data,
	dat_is_snpresp = dat_is_snpresp,
	is_fwd_snoop = is_fwd_snoop,
}
