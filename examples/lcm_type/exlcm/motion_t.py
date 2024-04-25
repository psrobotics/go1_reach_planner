"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class motion_t(object):
    __slots__ = ["timestamp", "latency", "rb_name", "position", "orientation", "enabled"]

    __typenames__ = ["int64_t", "int64_t", "string", "double", "double", "boolean"]

    __dimensions__ = [None, None, None, [3], [4], None]

    def __init__(self):
        self.timestamp = 0
        self.latency = 0
        self.rb_name = ""
        self.position = [ 0.0 for dim0 in range(3) ]
        self.orientation = [ 0.0 for dim0 in range(4) ]
        self.enabled = False

    def encode(self):
        buf = BytesIO()
        buf.write(motion_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qq", self.timestamp, self.latency))
        __rb_name_encoded = self.rb_name.encode('utf-8')
        buf.write(struct.pack('>I', len(__rb_name_encoded)+1))
        buf.write(__rb_name_encoded)
        buf.write(b"\0")
        buf.write(struct.pack('>3d', *self.position[:3]))
        buf.write(struct.pack('>4d', *self.orientation[:4]))
        buf.write(struct.pack(">b", self.enabled))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != motion_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return motion_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = motion_t()
        self.timestamp, self.latency = struct.unpack(">qq", buf.read(16))
        __rb_name_len = struct.unpack('>I', buf.read(4))[0]
        self.rb_name = buf.read(__rb_name_len)[:-1].decode('utf-8', 'replace')
        self.position = struct.unpack('>3d', buf.read(24))
        self.orientation = struct.unpack('>4d', buf.read(32))
        self.enabled = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if motion_t in parents: return 0
        tmphash = (0xd0191d36c049f797) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if motion_t._packed_fingerprint is None:
            motion_t._packed_fingerprint = struct.pack(">Q", motion_t._get_hash_recursive([]))
        return motion_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", motion_t._get_packed_fingerprint())[0]
