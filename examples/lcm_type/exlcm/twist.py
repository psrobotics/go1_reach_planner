"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class twist(object):
    __slots__ = ["timestamp", "enabled", "v_x", "v_y", "v_z", "v_roll", "v_pitch", "v_yaw"]

    __typenames__ = ["int64_t", "boolean", "double", "double", "double", "double", "double", "double"]

    __dimensions__ = [None, None, None, None, None, None, None, None]

    def __init__(self):
        self.timestamp = 0
        self.enabled = False
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
        self.v_roll = 0.0
        self.v_pitch = 0.0
        self.v_yaw = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(twist._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qbdddddd", self.timestamp, self.enabled, self.v_x, self.v_y, self.v_z, self.v_roll, self.v_pitch, self.v_yaw))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != twist._get_packed_fingerprint():
            raise ValueError("Decode error")
        return twist._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = twist()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.enabled = bool(struct.unpack('b', buf.read(1))[0])
        self.v_x, self.v_y, self.v_z, self.v_roll, self.v_pitch, self.v_yaw = struct.unpack(">dddddd", buf.read(48))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if twist in parents: return 0
        tmphash = (0x35a028dfbe72612c) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if twist._packed_fingerprint is None:
            twist._packed_fingerprint = struct.pack(">Q", twist._get_hash_recursive([]))
        return twist._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", twist._get_packed_fingerprint())[0]

