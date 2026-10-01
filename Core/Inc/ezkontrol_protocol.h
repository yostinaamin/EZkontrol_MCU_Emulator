#ifndef EZKONTROL_PROTOCOL_H
#define EZKONTROL_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * EZkontrol CAN protocol - shared addressing and ID construction reference.
 * This header is identical in EZkontrol_VCU and EZkontrol_MCU_Emulator.
 *
 * ------------------------------------------------------------------------
 * Extended CAN ID construction (29-bit, J1939-like PDU1 format)
 * ------------------------------------------------------------------------
 *
 *   bit 28:26  Priority        (3 bits)
 *   bit 25:24  DP/EDP          (2 bits, always 0 in this protocol)
 *   bit 23:16  PF              (PDU Format)
 *   bit 15:8   PS              (PDU Specific -> Destination Address, PF<240)
 *   bit  7:0   SA              (Source Address)
 *
 *   id = (priority & 0x7) << 26
 *      | (PF       & 0xFF) << 16
 *      | (PS       & 0xFF) << 8
 *      | (SA       & 0xFF)
 *
 * Verified against every ID in the EZkontrol documentation:
 *
 *   0x0C01EFD0  VCU  -> MCU1   priority=3 PF=0x01 PS=0xEF(MCU1) SA=0xD0(VCU)
 *   0x1801D0EF  MCU1 -> VCU    priority=6 PF=0x01 PS=0xD0(VCU)  SA=0xEF(MCU1)
 *   0x1802D0EF  MCU1 -> VCU    priority=6 PF=0x02 PS=0xD0(VCU)  SA=0xEF(MCU1)
 *   0x180117EF  MCU1 -> Meter  priority=6 PF=0x01 PS=0x17(Meter) SA=0xEF(MCU1)
 *   0x180217EF  MCU1 -> Meter  priority=6 PF=0x02 PS=0x17(Meter) SA=0xEF(MCU1)
 *
 * ------------------------------------------------------------------------
 * Node addresses (SA / PS values), as documented by the manufacturer
 * ------------------------------------------------------------------------
 */
#define EZK_ADDR_METER      0x17U
#define EZK_ADDR_VCU        0xD0U
#define EZK_ADDR_MCU1       0xEFU
#define EZK_ADDR_MCU2       0xF0U
#define EZK_ADDR_MCU3       0xF1U
#define EZK_ADDR_MCU4       0xF2U
#define EZK_ADDR_BMS1       0xF3U
#define EZK_ADDR_BMS2       0xF4U
#define EZK_ADDR_BMS3       0xF5U
#define EZK_ADDR_BMS4       0xF6U
#define EZK_ADDR_GLOBAL     0xFFU

/*
 * KNOWN MANUFACTURER PDF TYPO (see README for details):
 *   The Meter V1.1 PDF prints the Meter address as "23(0x11)" next to
 *   Message I. Decimal 23 is 0x17, NOT 0x11. The address table elsewhere
 *   in the same document, Message II, and the actual bus traffic on
 *   0x180117EF / 0x180217EF all use 0x17. EZK_ADDR_METER is therefore
 *   defined as 0x17. Do not "fix" this to 0x11 without new evidence.
 */

/* ------------------------------------------------------------------------
 * Extended CAN IDs actually used by this implementation
 * ------------------------------------------------------------------------ */
#define EZK_ID_VCU_TO_MCU1      0x0C01EFD0UL  /* VCU command -> MCU1 (also carries handshake AA reply) */
#define EZK_ID_MCU1_TO_VCU_1    0x1801D0EFUL  /* MCU1 feedback Msg I -> VCU (also carries handshake 55 request) */
#define EZK_ID_MCU1_TO_VCU_2    0x1802D0EFUL  /* MCU1 feedback Msg II -> VCU (temps/status/faults) */
#define EZK_ID_MCU1_TO_METER_1  0x180117EFUL  /* MCU1 -> Meter/dashboard Msg I (passively observable by VCU) */
#define EZK_ID_MCU1_TO_METER_2  0x180217EFUL  /* MCU1 -> Meter/dashboard Msg II (passively observable by VCU) */

/* ------------------------------------------------------------------------
 * Handshake
 * ------------------------------------------------------------------------
 * MCU1 -> VCU   ID=EZK_ID_MCU1_TO_VCU_1  DLC=8  data = 55 55 55 55 55 55 55 55
 * VCU  -> MCU1  ID=EZK_ID_VCU_TO_MCU1    DLC=8  data = AA AA AA AA AA AA AA AA
 *
 * The handshake reply and normal VCU commands share the same CAN ID
 * (EZK_ID_VCU_TO_MCU1). They are distinguished at the application layer
 * by checking whether ALL 8 payload bytes equal 0xAA.
 */
#define EZK_HANDSHAKE_REQUEST_BYTE   0x55U
#define EZK_HANDSHAKE_REPLY_BYTE     0xAAU

/* ------------------------------------------------------------------------
 * VCU -> MCU1 command frame (EZK_ID_VCU_TO_MCU1), period 50 ms, DLC 8
 * ------------------------------------------------------------------------
 * Bytes 0-1: target phase current, 0.1 A/bit, offset -3200A
 *            raw = physical_current_0p1A + 32000
 * Bytes 2-3: target speed, 1 rpm/bit, offset -32000rpm
 *            raw = speed_rpm + 32000
 * Byte 4:    bit0 RUN(1)/HALT(0), bit1 Speed(1)/Torque(0) mode, bits2-7 reserved
 * Bytes 5-6: reserved
 * Byte 7:    VCU life counter
 *
 * Signed convention: decoded_value = raw - 32000 for both current and speed.
 * Negative speed/current represents reverse-direction rotation/torque.
 */
#define EZK_CURRENT_OFFSET   32000
#define EZK_SPEED_OFFSET     32000
#define EZK_RAW_CLAMP_MIN    0
#define EZK_RAW_CLAMP_MAX    64000

static inline uint16_t ezk_encode_offset_0p1(int32_t physical_0p1_units)
{
    int32_t raw = physical_0p1_units + EZK_CURRENT_OFFSET;
    if (raw < EZK_RAW_CLAMP_MIN) raw = EZK_RAW_CLAMP_MIN;
    if (raw > EZK_RAW_CLAMP_MAX) raw = EZK_RAW_CLAMP_MAX;
    return (uint16_t)raw;
}

static inline int16_t ezk_decode_offset_0p1(uint16_t raw)
{
    return (int16_t)((int32_t)raw - EZK_CURRENT_OFFSET);
}

static inline uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void u16le_store(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

#ifdef __cplusplus
}
#endif

#endif /* EZKONTROL_PROTOCOL_H */
