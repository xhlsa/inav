#!/usr/bin/env python3
"""
Generate ExpressLRS reference vectors for rx_expresslrs_v{3,4}_unittest.cc.

This is an independent re-implementation of the transmitter side of
ExpressLRS, written from the upstream ExpressLRS sources (tags 3.6.4 and 4.1.0:
src/lib/FHSS/{FHSS.cpp,random.cpp}, src/lib/OTA/OTA.cpp, src/lib/CRC/crc.cpp,
src/include/crsf_protocol.h), NOT from the Betaflight/INAV receiver port it is
used to test. Run it to regenerate the headers:

    python3 rx_expresslrs_vectors_gen.py
"""

import hashlib

BIND_PHRASE = "lionbee"
FHSS_TEST_UID = [1, 2, 3, 4, 5, 6]

# 2.4GHz ISM domain (FHSS.cpp)
SX1280_XTAL_FREQ = 52000000
SX1280_FREQ_STEP = SX1280_XTAL_FREQ / (1 << 18)
FREQ_COUNT_24 = 80
FHSS_SEQUENCE_LEN = 256

CRSF_CHANNEL_VALUE_MIN = 172   # CRSF_CHANNEL_VALUE_STD_MIN in 4.x
CRSF_CHANNEL_VALUE_1000 = 191
CRSF_CHANNEL_VALUE_MID = 992
CRSF_CHANNEL_VALUE_2000 = 1792
CRSF_CHANNEL_VALUE_MAX = 1811  # CRSF_CHANNEL_VALUE_STD_MAX in 4.x

PACKET_TYPE_RCDATA = 0b00
PACKET_TYPE_SYNC = 0b10

SM_WIDE = 0
SM_HYBRID = 1

TLM_RATIO_NO_TLM = 1
TLM_RATIO_1_64 = 3

RATE_LORA_2G4_250HZ_V4 = 27     # expresslrs_RFrates_e in 4.1.0
RATE_INDEX_250HZ_V3 = 6         # ExpressLRS_AirRateConfig index in 3.6.4
HOP_INTERVAL_250HZ = 4


def uid_from_phrase(phrase):
    return list(hashlib.md5(('-DMY_BINDING_PHRASE="' + phrase + '"').encode()).digest()[:6])


# random.cpp
class Rng:
    def __init__(self, seed):
        self.seed = seed & 0xFFFFFFFF

    def rng(self):
        self.seed = (214013 * self.seed + 2531011) % 2147483648
        return (self.seed >> 16) & 0xFFFF

    def rng_n(self, maximum):
        return self.rng() % maximum


def uid_seed(uid, version):
    return ((uid[2] << 24) + (uid[3] << 16) + (uid[4] << 8) + (uid[5] ^ version)) & 0xFFFFFFFF


# FHSS.cpp FHSSrandomiseFHSSsequence / FHSSrandomiseFHSSsequenceBuild
def fhss_sequence(seed, freq_count, version):
    sync_channel = freq_count // 2 + (1 if version == 3 else 0)
    count = (FHSS_SEQUENCE_LEN // freq_count) * freq_count
    seq = [0] * FHSS_SEQUENCE_LEN
    rng = Rng(seed)
    for i in range(count):
        if i % freq_count == 0:
            seq[i] = sync_channel
        elif i % freq_count == sync_channel:
            seq[i] = 0
        else:
            seq[i] = i % freq_count
    for i in range(count):
        if i % freq_count != 0:
            offset = ((i // freq_count) * freq_count) & 0xFF
            rand = rng.rng_n(freq_count - 1) + 1
            seq[i], seq[offset + rand] = seq[offset + rand], seq[i]
    return seq, count


# crc.cpp Crc2Byte(14, ELRS_CRC14_POLY)
def crc14_table():
    tab = []
    for i in range(256):
        crc = (i << 6) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ (0x2E57 if crc & 0x2000 else 0)) & 0xFFFF
        tab.append(crc)
    return tab


CRC14_TAB = crc14_table()


def crc14(data, crc):
    crc &= 0xFFFF
    for b in data:
        crc = ((crc << 8) ^ CRC14_TAB[((crc >> 6) ^ b) & 0xFF]) & 0xFFFF
    return crc & 0x3FFF


# OTA.cpp OtaUpdateCrcInitFromUid
def crc_initializer(uid, version):
    init = (uid[4] << 8) | uid[5]
    if version == 3:
        return init ^ 3
    return init ^ (4 << 8)


# crsf_protocol.h
def fmap(x, in_min, in_max, out_min, out_max):
    # C integer division truncates towards zero; operands here are non-negative
    result = ((x - in_min) * (out_max - out_min) * 2 // (in_max - in_min) + out_min * 2 + 1) // 2
    return max(0, min(65535, result))


def crsf_to_uint10(val):
    return fmap(val, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 0, 1023)


def crsf_to_us(val):
    return fmap(val, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 988, 2012)


def uint10_to_crsf(val):
    return fmap(val, 0, 1023, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX)


def crsf_to_n(val, cnt):
    if val <= CRSF_CHANNEL_VALUE_1000:
        return 0
    if val >= CRSF_CHANNEL_VALUE_2000:
        return cnt - 1
    return (val - CRSF_CHANNEL_VALUE_1000) * cnt // (CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000 + 1)


def n_to_crsf(val, maximum):
    return val * (CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000) // maximum + CRSF_CHANNEL_VALUE_1000


def crsf_to_switch3b(ch):
    bin_size = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) // 6
    if ch < (CRSF_CHANNEL_VALUE_MID - bin_size // 4) or ch > (CRSF_CHANNEL_VALUE_MID + bin_size // 4):
        return crsf_to_n(ch, 6)
    return 7


def switch3b_to_crsf(val):
    if val == 0:
        return CRSF_CHANNEL_VALUE_1000
    if val == 5:
        return CRSF_CHANNEL_VALUE_2000
    if val in (6, 7):
        return CRSF_CHANNEL_VALUE_MID
    return val * 240 + 391


def crsf_to_bit(val):
    return 1 if val > CRSF_CHANNEL_VALUE_MID else 0


def hybrid_wide_nonce_to_switch_index(nonce):
    return ((nonce & 0b111) + ((nonce >> 3) & 0b1)) % 8


# OTA.cpp PackUInt11ToChannels4x10 with Decimate11to10_Limit
def pack_channels_4x10(ch):
    dest = [0] * 5
    d = 0
    dest_shift = 0
    for i in range(4):
        v = crsf_to_uint10(max(CRSF_CHANNEL_VALUE_MIN, min(CRSF_CHANNEL_VALUE_MAX, ch[i])))
        dest[d] |= (v << dest_shift) & 0xFF
        d += 1
        src_bits_left = 10 - 8 + dest_shift
        if d < 5:
            dest[d] = (v >> (10 - src_bits_left)) & 0xFF
        dest_shift = src_bits_left
    return dest


def finish_packet(pkt, uid, version, nonce, crc_high_inject=0):
    """OTA.cpp GeneratePacketCrcStd. pkt[0] holds the packet type."""
    ptype = pkt[0] & 0b11
    pkt[0] = ptype | ((crc_high_inject & 0x3F) << 2)
    if version == 3:
        crc = crc14(pkt[:7], crc_initializer(uid, 3))
    else:
        nonce_validator = 0 if ptype == PACKET_TYPE_SYNC else nonce
        crc = crc14(pkt[:7], crc_initializer(uid, 4) ^ nonce_validator)
    pkt[0] = ptype | (((crc >> 8) & 0x3F) << 2)
    pkt[7] = crc & 0xFF
    return pkt


def sync_packet(uid, version, nonce, fhss_index, switch_mode, tlm_ratio, model_id=0xFF):
    pkt = [0] * 8
    pkt[0] = PACKET_TYPE_SYNC
    pkt[1] = fhss_index
    pkt[2] = nonce
    new_tlm = tlm_ratio - TLM_RATIO_NO_TLM
    model_xor = (~model_id) & 0x3F
    if version == 3:
        pkt[3] = (switch_mode & 1) | ((new_tlm & 7) << 1) | ((RATE_INDEX_250HZ_V3 & 0xF) << 4)
        pkt[4] = uid[3]
        pkt[5] = uid[4]
        pkt[6] = uid[5] ^ model_xor
    else:
        pkt[3] = RATE_LORA_2G4_250HZ_V4
        pkt[4] = (switch_mode & 1) | ((new_tlm & 7) << 1)
        pkt[5] = uid[4]
        pkt[6] = uid[5] ^ model_xor
    return finish_packet(pkt, uid, version, nonce)


def rc_packet(uid, version, nonce, channels, switch_mode, hybrid_index, tlm_denom, ack=0, tx_power=3):
    """Returns (packet, expected channel dict in CRSF units for the channels this packet updates)."""
    pkt = [0] * 8
    pkt[0] = PACKET_TYPE_RCDATA
    pkt[1:6] = pack_channels_4x10(channels)
    armed_bit = crsf_to_bit(channels[4])
    expected = {}
    for i in range(4):
        expected[i] = uint10_to_crsf(crsf_to_uint10(max(CRSF_CHANNEL_VALUE_MIN, min(CRSF_CHANNEL_VALUE_MAX, channels[i]))))
    expected[4] = CRSF_CHANNEL_VALUE_2000 if armed_bit else CRSF_CHANNEL_VALUE_1000

    crc_inject = 0
    if switch_mode == SM_HYBRID:
        if hybrid_index == 6:
            value = crsf_to_n(channels[11], 16)
            expected[11] = n_to_crsf(value, 15)
        else:
            value = crsf_to_switch3b(channels[hybrid_index + 5])
            expected[hybrid_index + 5] = switch3b_to_crsf(value)
        switches = (ack << 6) | (hybrid_index << 3) | value
    else:
        idx = hybrid_wide_nonce_to_switch_index(nonce)
        if idx == 7:
            switches = (ack << 6) | tx_power
        else:
            if version == 3:
                low_res = 1 < tlm_denom < 8
                bins = 64 if low_res else 128
                value = crsf_to_n(channels[idx + 5], bins) & (bins - 1)
                switches = value | ((ack << 6) if low_res else 0)
                expected[idx + 5] = n_to_crsf(value, bins - 1)
            else:
                value = crsf_to_n(channels[idx + 5], 64) & 0x3F
                switches = (ack << 6) | value
                expected[idx + 5] = n_to_crsf(value, 63)
        if version == 3:
            crc_inject = (nonce % HOP_INTERVAL_250HZ) + 1

    pkt[6] = (switches & 0x7F) | (armed_bit << 7)
    return finish_packet(pkt, uid, version, nonce, crc_inject), expected


def c_array(values, per_line=16, fmt="{}"):
    lines = []
    for i in range(0, len(values), per_line):
        lines.append("    " + ", ".join(fmt.format(v) for v in values[i:i + per_line]) + ",")
    return "\n".join(lines)


def emit(version):
    uid = uid_from_phrase(BIND_PHRASE)
    seq, count = fhss_sequence(uid_seed(FHSS_TEST_UID, version), FREQ_COUNT_24, version)

    out = []
    out.append("// Generated by rx_expresslrs_vectors_gen.py from upstream ExpressLRS %s. Do not edit." % ("3.6.4" if version == 3 else "4.1.0"))
    out.append("#pragma once")
    out.append("")
    out.append('#define ELRS_TEST_BIND_PHRASE "%s"' % BIND_PHRASE)
    out.append("static const uint8_t elrsTestUid[6] = { %s };" % ", ".join("0x%02x" % b for b in uid))
    out.append("static const uint8_t elrsTestFhssUid[6] = { %s };" % ", ".join(str(b) for b in FHSS_TEST_UID))
    out.append("#define ELRS_TEST_FHSS_COUNT %d" % count)
    out.append("static const uint8_t elrsTestFhssSequence[%d] = {" % FHSS_SEQUENCE_LEN)
    out.append(c_array(seq, 20))
    out.append("};")
    out.append("")

    # Stick and switch positions, CRSF units
    channels = [
        [172, 992, 1811, 1400, 1792, 191, 500, 992, 1300, 1600, 1792, 1100, 992, 992, 992, 992],
        [1811, 300, 992, 1600, 191, 1792, 992, 700, 1000, 1500, 191, 1792, 992, 992, 992, 992],
    ]

    vectors = []
    sync_nonce = 8
    fhss_index = 5
    for mode in (SM_HYBRID, SM_WIDE):
        sync = sync_packet(uid, version, sync_nonce, fhss_index, mode, TLM_RATIO_1_64)
        for step in range(16):
            nonce = (sync_nonce + step) & 0xFF
            chans = channels[step % 2]
            pkt, expected = rc_packet(uid, version, nonce, chans, mode, step % 7, 64)
            vectors.append((mode, sync, nonce, pkt, expected))

    out.append("typedef struct {")
    out.append("    uint8_t switchMode;")
    out.append("    uint8_t sync[8];")
    out.append("    uint8_t nonce;")
    out.append("    uint8_t packet[8];")
    out.append("    uint16_t expectedUs[16];   // 0 = channel not carried by this packet")
    out.append("} elrsTestRcVector_t;")
    out.append("")
    out.append("#define ELRS_TEST_SYNC_NONCE %d" % sync_nonce)
    out.append("#define ELRS_TEST_SYNC_FHSS_INDEX %d" % fhss_index)
    out.append("static const elrsTestRcVector_t elrsTestRcVectors[] = {")
    for mode, sync, nonce, pkt, expected in vectors:
        us = [crsf_to_us(expected[c]) if c in expected else 0 for c in range(16)]
        out.append("    { %d, { %s }, %d, { %s }, { %s } }," % (
            mode,
            ", ".join("0x%02x" % b for b in sync),
            nonce,
            ", ".join("0x%02x" % b for b in pkt),
            ", ".join(str(v) for v in us)))
    out.append("};")
    out.append("")
    return "\n".join(out)


if __name__ == "__main__":
    for version in (3, 4):
        with open("rx_expresslrs_vectors_v%d.h" % version, "w") as f:
            f.write(emit(version))
