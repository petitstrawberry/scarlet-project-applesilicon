#!/usr/bin/env python3
"""Tests for the fixed J293 Type-C DCPext payload patch."""

import pathlib
import shutil
import tempfile
import unittest

import apple_dcpext_dtb


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
BASE_DTB = PROJECT_DIR / "m1n1" / "payloads" / "dtb" / "t8103-j293.dtb"
HAVE_DTC = all(shutil.which(tool) for tool in ("dtc", "fdtoverlay", "fdtget"))


@unittest.skipUnless(BASE_DTB.is_file() and HAVE_DTC, "J293 DTB or dtc tools unavailable")
class AppleDcpextDtbTests(unittest.TestCase):
    def test_patch_adds_complete_j293_topology(self):
        patched, changed = apple_dcpext_dtb.patch_dtb_bytes(BASE_DTB.read_bytes())

        self.assertTrue(changed)
        self.assertEqual(BASE_DTB.read_bytes()[:4], apple_dcpext_dtb.FDT_MAGIC)
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "patched.dtb"
            output.write_bytes(patched)
            self.assertTrue(apple_dcpext_dtb.dtb_has_dcpext(output))

    def test_patch_is_idempotent(self):
        patched, changed = apple_dcpext_dtb.patch_dtb_bytes(BASE_DTB.read_bytes())
        self.assertTrue(changed)

        second, changed_again = apple_dcpext_dtb.patch_dtb_bytes(patched)

        self.assertFalse(changed_again)
        self.assertEqual(second, patched)

    def test_payload_patch_preserves_m1n1_and_tail(self):
        prefix = b"m1n1-prefix"
        tail = b"compressed-u-boot-tail"
        payload = prefix + BASE_DTB.read_bytes() + tail

        patched, changed = apple_dcpext_dtb.patch_payload_bytes(payload)

        self.assertTrue(changed)
        self.assertTrue(patched.startswith(prefix + apple_dcpext_dtb.FDT_MAGIC))
        self.assertTrue(patched.endswith(tail))
        dtb_offset = len(prefix)
        dtb_size = int.from_bytes(patched[dtb_offset + 4 : dtb_offset + 8], "big")
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "patched.dtb"
            output.write_bytes(patched[dtb_offset : dtb_offset + dtb_size])
            self.assertTrue(apple_dcpext_dtb.dtb_has_dcpext(output))

    def test_rejects_unsupported_machine(self):
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "patched.dtb"
            with self.assertRaisesRegex(
                apple_dcpext_dtb.DcpextDtbError,
                "unsupported for j313",
            ):
                apple_dcpext_dtb.patch_dtb_file(BASE_DTB, output, "j313")


if __name__ == "__main__":
    unittest.main()
