"""UBX protocol parser for u-blox GNSS receivers.

Provides frame-level reading from a serial stream, checksum verification,
and typed parse functions for NAV-PVT, NAV-HPPOSLLH, and NAV-RELPOSNED messages.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

import serial

# UBX sync bytes
UBX_SYNC_1 = 0xB5
UBX_SYNC_2 = 0x62

# NAV class
UBX_NAV_CLASS = 0x01
UBX_NAV_PVT_ID = 0x07
UBX_NAV_HPPOSLLH_ID = 0x14
UBX_NAV_RELPOSNED_ID = 0x3C

# Expected payload sizes
NAV_PVT_PAYLOAD_SIZE = 92
NAV_HPPOSLLH_PAYLOAD_SIZE = 36
NAV_RELPOSNED_V0_PAYLOAD_SIZE = 40
NAV_RELPOSNED_V1_PAYLOAD_SIZE = 64


@dataclass
class UbxFrame:
    """Raw UBX frame with class, ID, and payload."""
    msg_class: int
    msg_id: int
    payload: bytes


@dataclass
class NavPvt:
    """Parsed NAV-PVT message fields."""
    itow: int       # GPS time of week [ms]
    year: int
    month: int
    day: int
    hour: int
    minute: int
    sec: int
    valid: int      # Validity flags
    t_acc: int      # Time accuracy estimate [ns]
    nano: int       # Fraction of second [ns]
    fix_type: int   # GNSS fix type (0-5)
    flags: int      # Fix status flags (includes carrSoln in bits 7..6)
    flags2: int     # Additional flags
    num_sv: int     # Number of SVs used
    lon: int        # Longitude [deg * 1e-7]
    lat: int        # Latitude [deg * 1e-7]
    height: int     # Height above ellipsoid [mm]
    h_msl: int      # Height above mean sea level [mm]
    h_acc: int      # Horizontal accuracy estimate [mm]
    v_acc: int      # Vertical accuracy estimate [mm]
    vel_n: int      # NED north velocity [mm/s]
    vel_e: int      # NED east velocity [mm/s]
    vel_d: int      # NED down velocity [mm/s]
    g_speed: int    # Ground speed [mm/s]
    head_mot: int   # Heading of motion [deg * 1e-5]
    s_acc: int      # Speed accuracy estimate [mm/s]
    head_acc: int   # Heading accuracy estimate [deg * 1e-5]
    p_dop: int      # Position DOP [1/0.01]
    flags3: int     # Additional flags
    head_veh: int   # Heading of vehicle [deg * 1e-5]
    mag_dec: int    # Magnetic declination [deg * 1e-2]
    mag_acc: int    # Magnetic declination accuracy [deg * 1e-2]
    # Derived fields
    fix_status: int   # 0=no carrier, 1=float, 2=fixed
    lon_deg: float    # Longitude [deg]
    lat_deg: float    # Latitude [deg]


@dataclass
class NavHpposllh:
    """Parsed NAV-HPPOSLLH message fields."""
    version: int
    flags: int       # bit0: invalidLlh
    itow: int        # GPS time of week [ms]
    lon: int         # Longitude [deg * 1e-7]
    lat: int         # Latitude [deg * 1e-7]
    height: int      # Height above ellipsoid [mm]
    h_msl: int       # Height above mean sea level [mm]
    lon_hp: int      # HP longitude component [deg * 1e-9]
    lat_hp: int      # HP latitude component [deg * 1e-9]
    height_hp: int   # HP height component [mm * 0.1]
    h_msl_hp: int    # HP hMSL component [mm * 0.1]
    h_acc: int       # Horizontal accuracy [mm * 0.1]
    v_acc: int       # Vertical accuracy [mm * 0.1]


@dataclass
class NavRelposned:
    """Parsed NAV-RELPOSNED message fields."""
    version: int
    ref_station_id: int
    itow: int            # GPS time of week [ms]
    rel_pos_n: int       # North component [cm]
    rel_pos_e: int       # East component [cm]
    rel_pos_d: int       # Down component [cm]
    rel_pos_length: int  # Length of vector [cm] (v1 only, 0 for v0)
    rel_pos_heading: int # Heading of vector [deg * 1e-5] (v1 only, 0 for v0)
    rel_pos_hpn: int     # HP North [mm * 0.1]
    rel_pos_hpe: int     # HP East [mm * 0.1]
    rel_pos_hpd: int     # HP Down [mm * 0.1]
    rel_pos_hp_length: int  # HP Length [mm * 0.1] (v1 only, 0 for v0)
    acc_n: int           # Accuracy North [mm * 0.1]
    acc_e: int           # Accuracy East [mm * 0.1]
    acc_d: int           # Accuracy Down [mm * 0.1]
    acc_length: int      # Accuracy Length [mm * 0.1] (v1 only, 0 for v0)
    acc_heading: int     # Accuracy Heading [deg * 1e-5] (v1 only, 0 for v0)
    flags: int           # Status flags
    # Derived fields
    fix_status: int      # 0=no carrier, 1=float, 2=fixed
    heading_rad: float   # ENU yaw = atan2(N, E) [rad]
    heading_deg: float   # ENU yaw [deg]
    qgc_heading: float   # NED heading (CW from North) [rad]


def compute_ubx_checksum(data: bytes) -> tuple[int, int]:
    """Compute UBX Fletcher-8 checksum over class+id+length+payload."""
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def read_ubx_frame(ser: serial.Serial) -> Optional[UbxFrame]:
    """Read a UBX frame after the first sync byte (0xB5) has been consumed.

    Reads sync2, class, ID, length, payload, and checksum.
    Returns None if the frame is invalid or incomplete.
    """
    sync2 = ser.read(1)
    if len(sync2) < 1 or sync2[0] != UBX_SYNC_2:
        return None

    header = ser.read(2)  # class + ID
    if len(header) < 2:
        return None

    length_bytes = ser.read(2)
    if len(length_bytes) < 2:
        return None
    length = struct.unpack('<H', length_bytes)[0]

    payload = ser.read(length)
    if len(payload) < length:
        return None

    checksum = ser.read(2)
    if len(checksum) < 2:
        return None

    # Verify checksum over class+id+length+payload
    ck_a, ck_b = compute_ubx_checksum(header + length_bytes + payload)
    if ck_a != checksum[0] or ck_b != checksum[1]:
        return None

    return UbxFrame(msg_class=header[0], msg_id=header[1], payload=bytes(payload))


def _extract_carr_soln(flags: int, shift: int) -> int:
    """Extract carrSoln field from flags and return fix_status (0/1/2)."""
    carr_soln = (flags >> shift) & 0x03
    if carr_soln == 2:
        return 2  # RTK fixed
    elif carr_soln == 1:
        return 1  # RTK float
    return 0      # No carrier phase solution


# ---- NAV-PVT payload format ----
# <  = little-endian
# I  = iTOW (U4)
# H  = year (U2)
# 6B = month, day, hour, min, sec, valid (U1 each)
# I  = tAcc (U4)
# i  = nano (I4)
# 4B = fixType, flags, flags2, numSV (U1 each)
# 4i = lon, lat, height, hMSL (I4 each)
# 2I = hAcc, vAcc (U4 each)
# 5i = velN, velE, velD, gSpeed, headMot (I4 each)
# 2I = sAcc, headAcc (U4 each)
# 2H = pDOP, flags3 (U2 each)
# 4x = reserved
# i  = headVeh (I4)
# h  = magDec (I2)
# H  = magAcc (U2)
_NAV_PVT_FMT = '<IH6BIi4B4i2I5i2I2H4xihH'


def parse_nav_pvt(payload: bytes) -> Optional[NavPvt]:
    """Parse NAV-PVT payload into a NavPvt dataclass."""
    if len(payload) < NAV_PVT_PAYLOAD_SIZE:
        return None

    fields = struct.unpack_from(_NAV_PVT_FMT, payload)
    (itow, year, month, day, hour, minute, sec, valid,
     t_acc, nano, fix_type, flags, flags2, num_sv,
     lon, lat, height, h_msl, h_acc, v_acc,
     vel_n, vel_e, vel_d, g_speed, head_mot,
     s_acc, head_acc, p_dop, flags3,
     head_veh, mag_dec, mag_acc) = fields

    fix_status = _extract_carr_soln(flags, 6)
    lon_deg = lon * 1e-7
    lat_deg = lat * 1e-7

    return NavPvt(
        itow=itow, year=year, month=month, day=day,
        hour=hour, minute=minute, sec=sec, valid=valid,
        t_acc=t_acc, nano=nano, fix_type=fix_type,
        flags=flags, flags2=flags2, num_sv=num_sv,
        lon=lon, lat=lat, height=height, h_msl=h_msl,
        h_acc=h_acc, v_acc=v_acc,
        vel_n=vel_n, vel_e=vel_e, vel_d=vel_d,
        g_speed=g_speed, head_mot=head_mot,
        s_acc=s_acc, head_acc=head_acc,
        p_dop=p_dop, flags3=flags3,
        head_veh=head_veh, mag_dec=mag_dec, mag_acc=mag_acc,
        fix_status=fix_status, lon_deg=lon_deg, lat_deg=lat_deg,
    )


# ---- NAV-HPPOSLLH payload format ----
# B   = version (U1)
# 2x  = reserved
# B   = flags (X1)
# I   = iTOW (U4)
# 4i  = lon, lat, height, hMSL (I4 each)
# 4b  = lonHp, latHp, heightHp, hMSLHp (I1 each)
# 2I  = hAcc, vAcc (U4 each)
_NAV_HPPOSLLH_FMT = '<B2xBI4i4b2I'


def parse_nav_hpposllh(payload: bytes) -> Optional[NavHpposllh]:
    """Parse NAV-HPPOSLLH payload into a NavHpposllh dataclass."""
    if len(payload) < NAV_HPPOSLLH_PAYLOAD_SIZE:
        return None

    fields = struct.unpack_from(_NAV_HPPOSLLH_FMT, payload)
    (version, flags, itow,
     lon, lat, height, h_msl,
     lon_hp, lat_hp, height_hp, h_msl_hp,
     h_acc, v_acc) = fields

    return NavHpposllh(
        version=version, flags=flags, itow=itow,
        lon=lon, lat=lat, height=height, h_msl=h_msl,
        lon_hp=lon_hp, lat_hp=lat_hp,
        height_hp=height_hp, h_msl_hp=h_msl_hp,
        h_acc=h_acc, v_acc=v_acc,
    )


# ---- NAV-RELPOSNED v1 payload format (64 bytes) ----
# B   = version (U1)
# x   = reserved
# H   = refStationId (U2)
# I   = iTOW (U4)
# 5i  = relPosN, relPosE, relPosD, relPosLength, relPosHeading (I4 each)
# 4x  = reserved
# 4b  = relPosHPN, relPosHPE, relPosHPD, relPosHPLength (I1 each)
# 5I  = accN, accE, accD, accLength, accHeading (U4 each)
# 4x  = reserved
# I   = flags (X4)
_NAV_RELPOSNED_V1_FMT = '<BxHI5i4x4b5I4xI'

# ---- NAV-RELPOSNED v0 payload format (40 bytes) ----
# B   = version (U1)
# x   = reserved
# H   = refStationId (U2)
# I   = iTOW (U4)
# 3i  = relPosN, relPosE, relPosD (I4 each)
# 3b  = relPosHPN, relPosHPE, relPosHPD (I1 each)
# x   = reserved
# 3I  = accN, accE, accD (U4 each)
# I   = flags (X4)
_NAV_RELPOSNED_V0_FMT = '<BxHI3i3bx3II'


def parse_nav_relposned(payload: bytes) -> Optional[NavRelposned]:
    """Parse NAV-RELPOSNED payload (v0 or v1) into a NavRelposned dataclass."""
    import math

    if len(payload) >= NAV_RELPOSNED_V1_PAYLOAD_SIZE:
        # Version 1 (64-byte payload)
        fields = struct.unpack_from(_NAV_RELPOSNED_V1_FMT, payload)
        (version, ref_station_id, itow,
         rel_pos_n, rel_pos_e, rel_pos_d, rel_pos_length, rel_pos_heading,
         rel_pos_hpn, rel_pos_hpe, rel_pos_hpd, rel_pos_hp_length,
         acc_n, acc_e, acc_d, acc_length, acc_heading,
         flags) = fields
    elif len(payload) >= NAV_RELPOSNED_V0_PAYLOAD_SIZE:
        # Version 0 (40-byte payload)
        fields = struct.unpack_from(_NAV_RELPOSNED_V0_FMT, payload)
        (version, ref_station_id, itow,
         rel_pos_n, rel_pos_e, rel_pos_d,
         rel_pos_hpn, rel_pos_hpe, rel_pos_hpd,
         acc_n, acc_e, acc_d,
         flags) = fields
        rel_pos_length = 0
        rel_pos_heading = 0
        rel_pos_hp_length = 0
        acc_length = 0
        acc_heading = 0
    else:
        return None

    # carrSoln is at bits 4..3 of flags
    fix_status = _extract_carr_soln(flags, 3)

    # Compute heading from HP N/E components
    # Full N in cm: rel_pos_n + rel_pos_hpn * 0.01
    # Full E in cm: rel_pos_e + rel_pos_hpe * 0.01
    n_full = rel_pos_n + rel_pos_hpn * 0.01
    e_full = rel_pos_e + rel_pos_hpe * 0.01

    # ENU yaw: angle from East axis, counter-clockwise
    heading_rad = math.atan2(n_full, e_full)
    heading_deg = math.degrees(heading_rad)

    # NED heading (clockwise from North)
    qgc_heading = -heading_rad + math.pi / 2.0
    if qgc_heading > math.pi:
        qgc_heading -= 2.0 * math.pi
    if qgc_heading < -math.pi:
        qgc_heading += 2.0 * math.pi

    return NavRelposned(
        version=version, ref_station_id=ref_station_id, itow=itow,
        rel_pos_n=rel_pos_n, rel_pos_e=rel_pos_e, rel_pos_d=rel_pos_d,
        rel_pos_length=rel_pos_length, rel_pos_heading=rel_pos_heading,
        rel_pos_hpn=rel_pos_hpn, rel_pos_hpe=rel_pos_hpe, rel_pos_hpd=rel_pos_hpd,
        rel_pos_hp_length=rel_pos_hp_length,
        acc_n=acc_n, acc_e=acc_e, acc_d=acc_d,
        acc_length=acc_length, acc_heading=acc_heading,
        flags=flags,
        fix_status=fix_status,
        heading_rad=heading_rad, heading_deg=heading_deg,
        qgc_heading=qgc_heading,
    )
