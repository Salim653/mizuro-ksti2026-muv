/* Minimal STORE-method zip writer (no dependencies, works under file://).
   Same code runs in the browser and in Node — `npm`-free unit test lives next to it.
   entries: [{ name: 'a/b.jpg', data: Uint8Array }]  ->  Blob (browser) / Uint8Array (Node) */
(function (root, factory) {
  var api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (typeof window === 'object') window.zipStore = api;
  if (typeof globalThis === 'object') globalThis.zipStore = api;
})(this, function () {
  'use strict';

  var CRC_T = (function () {
    var t = new Uint32Array(256);
    for (var n = 0; n < 256; n++) {
      var c = n;
      for (var k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
      t[n] = c >>> 0;
    }
    return t;
  })();

  function crc32(u8) {
    var c = 0xFFFFFFFF;
    for (var i = 0; i < u8.length; i++) c = CRC_T[(c ^ u8[i]) & 0xFF] ^ (c >>> 8);
    return (c ^ 0xFFFFFFFF) >>> 0;
  }

  function utf8(str) { return new TextEncoder().encode(str); }

  function dataUrlToU8(url) {
    var s = url.slice(url.indexOf(',') + 1);
    var bin = (typeof atob === 'function') ? atob(s) : Buffer.from(s, 'base64').toString('binary');
    var u = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
    return u;
  }

  /* entries -> [{name, data:Uint8Array, size, crc}] , plus the central-directory byte count */
  function build(parts, entries, when) {
    var enc = new TextEncoder();
    var d = when || new Date();
    var dosTime = (d.getHours() << 11) | (d.getMinutes() << 5) | (d.getSeconds() >> 1);
    var dosDate = ((Math.max(1980, d.getFullYear()) - 1980) << 9) | ((d.getMonth() + 1) << 5) | d.getDate();
    var offset = 0, cd = [], count = 0, cdSize = 0;

    entries.forEach(function (e) {
      var nm = enc.encode(e.name);
      var crc = crc32(e.data);
      var lh = new DataView(new ArrayBuffer(30));
      lh.setUint32(0, 0x04034b50, true);
      lh.setUint16(4, 20, true);            // version needed
      lh.setUint16(6, 0x0800, true);        // flags: bit 11 = UTF-8 names
      lh.setUint16(8, 0, true);             // method 0 = stored
      lh.setUint16(10, dosTime, true);
      lh.setUint16(12, dosDate, true);
      lh.setUint32(14, crc, true);
      lh.setUint32(18, e.data.length, true);
      lh.setUint32(22, e.data.length, true);
      lh.setUint16(26, nm.length, true);
      lh.setUint16(28, 0, true);
      parts.push(new Uint8Array(lh.buffer), nm, e.data);

      var ch = new DataView(new ArrayBuffer(46));
      ch.setUint32(0, 0x02014b50, true);
      ch.setUint16(4, 20, true);
      ch.setUint16(6, 20, true);
      ch.setUint16(8, 0x0800, true);
      ch.setUint16(10, 0, true);
      ch.setUint16(12, dosTime, true);
      ch.setUint16(14, dosDate, true);
      ch.setUint32(16, crc, true);
      ch.setUint32(20, e.data.length, true);
      ch.setUint32(24, e.data.length, true);
      ch.setUint16(28, nm.length, true);
      ch.setUint32(42, offset, true);
      cd.push(new Uint8Array(ch.buffer), nm);
      count++;
      offset += 30 + nm.length + e.data.length;
      cdSize += 46 + nm.length;
    });

    var eo = new DataView(new ArrayBuffer(22));
    eo.setUint32(0, 0x06054b50, true);
    eo.setUint16(8, count, true);
    eo.setUint16(10, count, true);
    eo.setUint32(12, cdSize, true);
    eo.setUint32(16, offset, true);
    cd.forEach(function (p) { parts.push(p); });
    parts.push(new Uint8Array(eo.buffer));
    return count;
  }

  return {
    crc32: crc32,
    utf8: utf8,
    dataUrlToU8: dataUrlToU8,
    /** entries -> Blob. Constructing a Blob can detach the views it is given, so callers
        must hand over data they will not reuse — the recorder re-decodes from base64
        on every export, which keeps repeated exports safe. */
    makeBlob: function (entries, when) {
      var parts = [];
      build(parts, entries, when);
      return new Blob(parts, { type: 'application/zip' });
    },
    /** entries -> Uint8Array (Node / tests) */
    makeBytes: function (entries, when) {
      var flat = [], total = 0;
      build(flat, entries, when);
      flat.forEach(function (p) { total += p.length; });
      var out = new Uint8Array(total), o = 0;
      flat.forEach(function (p) { out.set(p, o); o += p.length; });
      return out;
    }
  };
});
