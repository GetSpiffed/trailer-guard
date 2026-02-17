function decodeUplink(input) {
  const bytes = input.bytes || [];
  if (bytes.length < 11) {
    return {
      errors: ['Payload too short, expected 11 bytes'],
    };
  }

  const flags = bytes[0];
  const hasFix = (flags & 0x01) === 0x01;

  const latE7 = readInt32LE(bytes, 1);
  const lonE7 = readInt32LE(bytes, 5);
  const speedKmh = bytes[9];
  const battPct = bytes[10];

  return {
    data: {
      hasFix: hasFix,
      lat: hasFix ? latE7 / 1e7 : 0,
      lon: hasFix ? lonE7 / 1e7 : 0,
      speedKmh: hasFix ? speedKmh : 0,
      battPct: battPct,
      ageMsOptioneel: null,
    },
  };
}

function readInt32LE(bytes, offset) {
  const b0 = bytes[offset] & 0xff;
  const b1 = bytes[offset + 1] & 0xff;
  const b2 = bytes[offset + 2] & 0xff;
  const b3 = bytes[offset + 3] & 0xff;
  const value = (b0 | (b1 << 8) | (b2 << 16) | (b3 << 24));
  return value | 0;
}
