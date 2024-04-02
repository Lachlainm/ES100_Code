--[[
   SVFFI serial protocol for generator support
   See http://www.svffi.com/en/
--]]

local PARAM_TABLE_KEY = 42
local PARAM_TABLE_PREFIX = "EFI_SVF_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- bind a parameter to a variable given
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: EFI_SVF_ENABLE
  // @DisplayName: Generator SVFFI enable
  // @Description: Enable SVFFI generator support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
EFI_SVF_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  // @Param: EFI_SVF_ARMCHECK
  // @DisplayName: Generator SVFFI arming check
  // @Description: Check for Generator ARM state before arming
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
EFI_SVF_ARMCHECK = bind_add_param("ARMCHECK", 2, 1)

if EFI_SVF_ENABLE:get() ~= 1 then
   return
end

local auth_id = arming:get_aux_auth_id()
arming:set_aux_auth_failed(auth_id, "GEN: not in ARM state")


local uart = serial:find_serial(0) -- first scripting serial
if not uart then
   gcs:send_text(MAV_SEVERITY.ERROR, "GEN_SVF: unable to find serial port")
   return
end
uart:begin(115200)

local efi_backend = efi:get_backend(0)
if not efi_backend then
   gcs:send_text(MAV_SEVERITY.ERROR, "GEN_SVF: unable to find EFI backend")
   return
end

local function read_bytes(n)
   local ret = ""
   for _ = 1, n do
      ret = ret .. string.char(uart:read())
   end
   return ret
end

local auchCRCHi = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
}

local auchCRCLo = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
}

--[[
   calculate crc16
--]]
local function get_crc16(s)
   local uchCRCHi = 0xFF
   local uchCRCLo = 0xFF
   for i = 1, #s do
      local b = string.byte(string.sub(s, i, i))
      local uIndex = uchCRCLo ~ b
      --gcs:send_text(MAV_SEVERITY.INFO, string.format("uIndex=%u", uIndex))
      uchCRCLo = uchCRCHi ~ auchCRCHi[uIndex+1]
      uchCRCHi = auchCRCLo[uIndex+1]
   end
   return (uchCRCHi << 8 | uchCRCLo)
end

local state = {}
state.last_read_us = uint32_t(0)
state.last_status = -1


--[[
   check for input and parse data
--]]
local function check_input()
   local n_bytes = uart:available():toint()
   --gcs:send_text(MAV_SEVERITY.INFO, string.format("n_bytes=%u %.2f", n_bytes, millis():tofloat()*0.001))
   if n_bytes < 31 then
      return
   end

   local s = read_bytes(n_bytes)
   local prefix, len = string.unpack("<HB", s, 1)
   if prefix ~= 0xa55a then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("bad prefix 0x%x", prefix))
      return
   end
   if len+5 ~= n_bytes then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("bad len %u %u", n_bytes, len))
      return
   end
   local crc = string.unpack("<H", s, 4+len)
   local s2 = string.sub(s,1,n_bytes-2)
   --gcs:send_text(MAV_SEVERITY.INFO, string.format("s2 n_bytes=%u len1=%u len2=%u", n_bytes, #s, #s2))
   local crc2 = get_crc16(s2)
   if crc ~= crc2 then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("bad crc %x %x", crc, crc2))
      return
   end

   state.version, state.rpm, state.throttle = string.unpack("<BHH", string.sub(s,4,8))
   state.voltage, state.current, state.runtime = string.unpack("<HHI", string.sub(s,9,16))
   state.maint_time, state.lock_time, state.status = string.unpack("<HHB", string.sub(s,17,21))
   state.alarm, state.fuellevel, state.cht1 = string.unpack("<HBH", string.sub(s,22,26))
   state.cht2, state.pcb_temp = string.unpack("<HB", string.sub(s,27,29))

   if state.status ~= state.last_status then
      state.last_status = state.status
      local states = {"STOP", "IDLE", "RUN", "3", "CHARGE" }
      local name = states[state.status+1]
      if not name then
         name = string.format("Unknown%u", state.status)
      end
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("Generator state: %s", name))
      if name ~= "RUN" and EFI_SVF_ARMCHECK:get() == 1 then
         arming:set_aux_auth_failed(auth_id, string.format("GEN: not in ARM state (%s)", name))
      else
         arming:set_aux_auth_passed(auth_id)
      end

   end

   state.last_read_us = micros()
end

--[[
   update EFI state
--]]
local function update_EFI()
   if state.last_read_us == uint32_t(0) then
      return
   end
   local cylinder_state = Cylinder_Status()
   local efi_state = EFI_State()
   local C_TO_KELVIN = 273.2

   cylinder_state:cylinder_head_temperature(state.cht1+C_TO_KELVIN)
   efi_state:engine_speed_rpm(state.rpm)

   efi_state:throttle_position_percent(state.throttle)
   efi_state:ignition_voltage(state.voltage*0.1)

   efi_state:cylinder_status(cylinder_state)
   efi_state:last_updated_ms(millis())

   -- Set the EFI_State into the EFI scripting driver
   efi_backend:handle_scripting(efi_state)

   gcs:send_named_float('GEN_VOLT', state.voltage*0.1)
   gcs:send_named_float('GEN_AMPS', state.current*0.1)
   gcs:send_named_float('GEN_STAT', state.status)
   logger.write('SVF','Curr,Volt,Status', 'ffB',
                state.current*0.1,
                state.voltage*0.1,
                state.status)


end


--[[
   main update function
--]]
local function update()
   check_input()
   update_EFI()

   return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "GEN_SVF: loaded")

return update()
