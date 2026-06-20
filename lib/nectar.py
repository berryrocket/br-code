########################################
# BerryRocket - NectarMC frame encoder
# Protocol: https://github.com/mlavardin/NectarMC/blob/master/DOCUMENTATION/FRAME_FORMAT.md
# Licence CC-BY-NC-SA
# See https://creativecommons.org/licenses/by-nc-sa/4.0
########################################

MAGIC = 0xEB

SSID_FX      = 0
SSID_MF      = 1
SSID_BALLOON = 2
SSID_OTHER   = 3

def crc16_ccitt(data, poly=0x1021, init=0xFFFF):
    """CRC16-CCITT (poly=0x1021, init=0xFFFF, no reflection) over the given bytes."""
    crc = init
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def build_frame(ssid_type, ssid_num, apid, payload):
    """Assemble a Nectar frame:
        [MAGIC=0xEB][Id_mission LE 2B][payload_size 1B][payload N][CRC16 LE 2B]
    Id_mission = (SSID << 6) | APID  with  SSID = (TYPE << 8) | NUM."""
    if len(payload) > 255:
        raise ValueError("payload > 255 bytes")
    if not (0 <= ssid_type <= 3):
        raise ValueError("ssid_type out of range 0..3")
    if not (0 <= ssid_num <= 255):
        raise ValueError("ssid_num out of range 0..255")
    if not (0 <= apid <= 63):
        raise ValueError("apid out of range 0..63")

    ssid = (ssid_type << 8) | ssid_num
    id_mission = ((ssid << 6) | apid) & 0xFFFF

    header = bytes((
        MAGIC,
        id_mission & 0xFF,
        (id_mission >> 8) & 0xFF,
        len(payload),
    ))
    body = header + bytes(payload)
    crc = crc16_ccitt(body)
    return body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
