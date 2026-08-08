#!/usr/bin/env python3
"""Tests for the fixed J293 Type-C DCPext payload patch."""

import pathlib
import shutil
import tempfile
import unittest

import apple_dcpext_dtb


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
BASE_DTB = PROJECT_DIR / "m1n1" / "payloads" / "dtb" / "t8103-j293.dtb"
HAVE_DTC = all(
    shutil.which(tool) for tool in ("dtc", "fdtoverlay", "fdtget", "fdtput")
)


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
            dcpext = apple_dcpext_dtb._phandle(
                output,
                apple_dcpext_dtb.DCPEXT_PATH,
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.TYPEC0_PATH,
                    "displayport",
                ),
                [dcpext],
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.TYPEC1_PATH,
                    "displayport",
                ),
                [dcpext],
            )
            for name in apple_dcpext_dtb.LEGACY_FIXED_ROUTE_PROPERTIES:
                self.assertFalse(
                    apple_dcpext_dtb._has_property(
                        output,
                        apple_dcpext_dtb.DCPEXT_PATH,
                        name,
                    )
                )

    def test_patch_is_idempotent(self):
        patched, changed = apple_dcpext_dtb.patch_dtb_bytes(BASE_DTB.read_bytes())
        self.assertTrue(changed)

        second, changed_again = apple_dcpext_dtb.patch_dtb_bytes(patched)

        self.assertFalse(changed_again)
        self.assertEqual(second, patched)

    def test_patch_adds_sio_dpaudio_topology(self):
        patched, changed = apple_dcpext_dtb.patch_dtb_bytes(BASE_DTB.read_bytes())
        self.assertTrue(changed)

        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "patched.dtb"
            output.write_bytes(patched)
            sio = apple_dcpext_dtb._phandle(output, apple_dcpext_dtb.SIO_PATH)
            sio_mbox = apple_dcpext_dtb._phandle(
                output,
                apple_dcpext_dtb.SIO_MBOX_PATH,
            )
            dpaudio1_power = apple_dcpext_dtb._phandle(
                output,
                apple_dcpext_dtb.DPAUDIO1_POWER_PATH,
            )

            self.assertEqual(
                apple_dcpext_dtb._string_property(
                    output,
                    apple_dcpext_dtb.ALIASES_PATH,
                    "sio",
                ),
                apple_dcpext_dtb.SIO_PATH,
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.SIO_PATH,
                    "mboxes",
                ),
                [sio_mbox],
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.DPAUDIO1_PATH,
                    "dmas",
                ),
                [sio, 0x66],
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.DPAUDIO1_PATH,
                    "power-domains",
                ),
                [dpaudio1_power],
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.DPAUDIO1_PATH,
                    "reset-domains",
                ),
                [dpaudio1_power],
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.DPAUDIO1_PATH,
                    "resets",
                ),
                [dpaudio1_power],
            )

            dpaudio1_endpoint = apple_dcpext_dtb._phandle(
                output,
                apple_dcpext_dtb.DPAUDIO1_ENDPOINT_PATH,
            )
            dcpext_endpoint = apple_dcpext_dtb._phandle(
                output,
                apple_dcpext_dtb.DCPEXT_AUDIO_ENDPOINT_PATH,
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.DPAUDIO1_ENDPOINT_PATH,
                    "remote-endpoint",
                ),
                [dcpext_endpoint],
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.DCPEXT_AUDIO_ENDPOINT_PATH,
                    "remote-endpoint",
                ),
                [dpaudio1_endpoint],
            )

    def test_patch_preserves_m1n1_sio_firmware_properties(self):
        patched, changed = apple_dcpext_dtb.patch_dtb_bytes(BASE_DTB.read_bytes())
        self.assertTrue(changed)

        with tempfile.TemporaryDirectory() as directory:
            m1n1_dtb = pathlib.Path(directory) / "m1n1.dtb"
            output = pathlib.Path(directory) / "output.dtb"
            m1n1_dtb.write_bytes(patched)
            sio = apple_dcpext_dtb._phandle(
                m1n1_dtb,
                apple_dcpext_dtb.SIO_PATH,
            )
            fw_region_path = "/reserved-memory/sio-firmware-data@900000000"

            # These are deliberately representative values.  m1n1 owns the
            # actual reserved-memory phandles, IOVAs, and firmware params.
            apple_dcpext_dtb._run(
                [
                    apple_dcpext_dtb._tool("fdtput"),
                    "-c",
                    m1n1_dtb,
                    fw_region_path,
                ]
            )
            for name, values in (
                ("phandle", ["0x1234"]),
                ("reg", ["0x9", "0", "0", "0x4000"]),
                (
                    "iommu-addresses",
                    [f"0x{sio:x}", "0", "0x100000", "0", "0x4000"],
                ),
            ):
                apple_dcpext_dtb._run(
                    [
                        apple_dcpext_dtb._tool("fdtput"),
                        "-t",
                        "x",
                        m1n1_dtb,
                        fw_region_path,
                        name,
                        *values,
                    ]
                )
            apple_dcpext_dtb._run(
                [
                    apple_dcpext_dtb._tool("fdtput"),
                    "-t",
                    "x",
                    m1n1_dtb,
                    apple_dcpext_dtb.SIO_PATH,
                    "memory-region",
                    "0x1234",
                ]
            )
            apple_dcpext_dtb._run(
                [
                    apple_dcpext_dtb._tool("fdtput"),
                    "-t",
                    "x",
                    m1n1_dtb,
                    apple_dcpext_dtb.SIO_PATH,
                    "apple,sio-firmware-params",
                    "0x10",
                    "0x20",
                ]
            )
            apple_dcpext_dtb._run(
                [
                    apple_dcpext_dtb._tool("fdtput"),
                    "-t",
                    "s",
                    m1n1_dtb,
                    apple_dcpext_dtb.SIO_PATH,
                    "status",
                    "okay",
                ]
            )

            changed_again = apple_dcpext_dtb.patch_dtb_file(m1n1_dtb, output)

            self.assertFalse(changed_again)
            self.assertEqual(output.read_bytes(), m1n1_dtb.read_bytes())
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.SIO_PATH,
                    "memory-region",
                ),
                [0x1234],
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    apple_dcpext_dtb.SIO_PATH,
                    "apple,sio-firmware-params",
                ),
                [0x10, 0x20],
            )
            self.assertEqual(
                apple_dcpext_dtb._cells_property(
                    output,
                    fw_region_path,
                    "iommu-addresses",
                ),
                [sio, 0, 0x100000, 0, 0x4000],
            )

    def test_patch_migrates_legacy_atc1_fixed_route(self):
        patched, changed = apple_dcpext_dtb.patch_dtb_bytes(BASE_DTB.read_bytes())
        self.assertTrue(changed)

        with tempfile.TemporaryDirectory() as directory:
            legacy = pathlib.Path(directory) / "legacy.dtb"
            migrated = pathlib.Path(directory) / "migrated.dtb"
            legacy.write_bytes(patched)
            atcphy1 = apple_dcpext_dtb._phandle(
                legacy,
                apple_dcpext_dtb.ATCPHY1_PATH,
            )
            apple_dcpext_dtb._run(
                [
                    apple_dcpext_dtb._tool("fdtput"),
                    "-t",
                    "x",
                    legacy,
                    apple_dcpext_dtb.DCPEXT_PATH,
                    "phys",
                    f"0x{atcphy1:x}",
                    "0x6",
                ]
            )
            apple_dcpext_dtb._run(
                [
                    apple_dcpext_dtb._tool("fdtput"),
                    "-t",
                    "s",
                    legacy,
                    apple_dcpext_dtb.DCPEXT_PATH,
                    "phy-names",
                    "dp-phy",
                ]
            )
            apple_dcpext_dtb._run(
                [
                    apple_dcpext_dtb._tool("fdtput"),
                    "-d",
                    legacy,
                    apple_dcpext_dtb.TYPEC0_PATH,
                    "displayport",
                ]
            )

            changed_again = apple_dcpext_dtb.patch_dtb_file(legacy, migrated)

            self.assertTrue(changed_again)
            self.assertTrue(apple_dcpext_dtb.dtb_has_dcpext(migrated))
            self.assertFalse(
                apple_dcpext_dtb._has_property(
                    migrated,
                    apple_dcpext_dtb.DCPEXT_PATH,
                    "phys",
                )
            )

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
