#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Patch J293 Type-C DCPext topology into an m1n1 payload DTB.

The stock J293 DTB already contains DCPext, its mailbox, and both DARTs, but
keeps them disabled and does not connect DCPext to the front Type-C port.
This helper applies the fixed fairydust-style J293 topology before stage-2
m1n1 consumes the DTB.  No information is taken from the live ADT.
"""

from __future__ import annotations

import argparse
import pathlib
import shutil
import subprocess
import sys
import tempfile
from typing import Any


FDT_MAGIC = b"\xd0\r\xfe\xed"
SUPPORTED_MACHINE = "j293"
PHY_TYPE_DP = 6

ALIASES_PATH = "/aliases"
SOC_PATH = "/soc"
DISPLAY_PATH = "/soc/display-subsystem"
DISP0_DART_PATH = "/soc/iommu@231304000"
DISPEXT0_DART_PATH = "/soc/iommu@271304000"
DCPEXT_DART_PATH = "/soc/iommu@27130c000"
DCPEXT_MBOX_PATH = "/soc/mbox@271c08000"
DCPEXT_PATH = "/soc/dcp@271c00000"
ATCPHY1_PATH = "/soc/phy@503000000"
ATCPHY1_XBAR_PATH = "/soc/mux@50304c000"
TYPEC1_PATH = "/soc/i2c@235010000/usb-pd@3f/connector"
ATC1_COMMON_PATH = "/soc/power-management@23b700000/power-controller@448"


class DcpextDtbError(RuntimeError):
    """Raised when the fixed J293 DCPext patch cannot be applied."""


def _run(
    cmd: list[str | pathlib.Path],
    *,
    check: bool = True,
    **kwargs: Any,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(part) for part in cmd],
        check=check,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        **kwargs,
    )


def _tool(name: str) -> str:
    path = shutil.which(name)
    if path is None:
        raise DcpextDtbError(f"{name} not found; install device-tree-compiler")
    return path


def _node_exists(dtb: pathlib.Path, path: str) -> bool:
    result = _run([_tool("fdtget"), "-l", dtb, path], check=False)
    return result.returncode == 0


def _property(
    dtb: pathlib.Path,
    path: str,
    name: str,
    value_type: str,
) -> list[str] | None:
    result = _run(
        [_tool("fdtget"), "-t", value_type, dtb, path, name],
        check=False,
    )
    if result.returncode != 0:
        return None
    return result.stdout.split()


def _has_property(dtb: pathlib.Path, path: str, name: str) -> bool:
    result = _run([_tool("fdtget"), dtb, path, name], check=False)
    return result.returncode == 0


def _string_property(dtb: pathlib.Path, path: str, name: str) -> str | None:
    values = _property(dtb, path, name, "s")
    return " ".join(values) if values is not None else None


def _cells_property(dtb: pathlib.Path, path: str, name: str) -> list[int] | None:
    values = _property(dtb, path, name, "x")
    if values is None:
        return None
    try:
        return [int(value, 16) for value in values]
    except ValueError as exc:
        raise DcpextDtbError(f"invalid cells in {path}.{name}") from exc


def _phandle(dtb: pathlib.Path, path: str) -> int:
    for name in ("phandle", "linux,phandle"):
        values = _cells_property(dtb, path, name)
        if values:
            return values[0]
    raise DcpextDtbError(f"guest DTB node has no phandle: {path}")


def _require_j293_nodes(dtb: pathlib.Path) -> None:
    for path in (
        ALIASES_PATH,
        DISPLAY_PATH,
        DISP0_DART_PATH,
        DISPEXT0_DART_PATH,
        DCPEXT_DART_PATH,
        DCPEXT_MBOX_PATH,
        DCPEXT_PATH,
        ATCPHY1_PATH,
        TYPEC1_PATH,
        ATC1_COMMON_PATH,
    ):
        if not _node_exists(dtb, path):
            raise DcpextDtbError(f"J293 guest DTB is missing {path}")


def dtb_has_dcpext(dtb: pathlib.Path) -> bool:
    """Return whether a DTB has the complete fixed J293 video topology."""
    try:
        _require_j293_nodes(dtb)
        dcpext = _phandle(dtb, DCPEXT_PATH)
        disp0_dart = _phandle(dtb, DISP0_DART_PATH)
        dispext0_dart = _phandle(dtb, DISPEXT0_DART_PATH)
        dcpext_dart = _phandle(dtb, DCPEXT_DART_PATH)
        dcpext_mbox = _phandle(dtb, DCPEXT_MBOX_PATH)
        atcphy1 = _phandle(dtb, ATCPHY1_PATH)
        xbar = _phandle(dtb, ATCPHY1_XBAR_PATH)
    except DcpextDtbError:
        return False

    expected_status = (DISPEXT0_DART_PATH, DCPEXT_DART_PATH, DCPEXT_MBOX_PATH, DCPEXT_PATH)
    return (
        _string_property(dtb, ALIASES_PATH, "dcpext") == DCPEXT_PATH
        and _cells_property(dtb, TYPEC1_PATH, "displayport") == [dcpext]
        and _cells_property(dtb, DISPLAY_PATH, "iommus")
        == [disp0_dart, 0, dispext0_dart, 0]
        and all(_string_property(dtb, path, "status") == "okay" for path in expected_status)
        and _string_property(dtb, DCPEXT_PATH, "apple,connector-type") == "DP"
        and _cells_property(dtb, DCPEXT_PATH, "apple,dptx-phy") == [1]
        and _cells_property(dtb, DCPEXT_PATH, "phys") == [atcphy1, PHY_TYPE_DP]
        and _string_property(dtb, DCPEXT_PATH, "phy-names") == "dp-phy"
        and _cells_property(dtb, DCPEXT_PATH, "mux-controls") == [xbar, 0]
        and _string_property(dtb, DCPEXT_PATH, "mux-control-names") == "dp-xbar"
        and _cells_property(dtb, DCPEXT_PATH, "mux-index") == [0]
        and _string_property(dtb, ATCPHY1_XBAR_PATH, "compatible")
        == "apple,t8103-display-crossbar"
        and _cells_property(dtb, ATCPHY1_XBAR_PATH, "#mux-control-cells") == [1]
        and _cells_property(dtb, DCPEXT_PATH, "iommus") == [dcpext_dart, 0]
        and _cells_property(dtb, DCPEXT_PATH, "mboxes") == [dcpext_mbox]
        and _has_property(dtb, ATC1_COMMON_PATH, "apple,always-on")
    )


def _overlay_dts(dtb: pathlib.Path) -> str:
    _require_j293_nodes(dtb)
    dcpext = _phandle(dtb, DCPEXT_PATH)
    disp0_dart = _phandle(dtb, DISP0_DART_PATH)
    dispext0_dart = _phandle(dtb, DISPEXT0_DART_PATH)
    atcphy1 = _phandle(dtb, ATCPHY1_PATH)
    atc1_power = _cells_property(dtb, ATCPHY1_PATH, "power-domains")
    if not atc1_power:
        raise DcpextDtbError(f"guest DTB is missing {ATCPHY1_PATH}.power-domains")

    return f"""/dts-v1/;
/plugin/;

/ {{
    fragment@0 {{
        target-path = "{ALIASES_PATH}";
        __overlay__ {{
            dcpext = "{DCPEXT_PATH}";
        }};
    }};

    fragment@1 {{
        target-path = "{TYPEC1_PATH}";
        __overlay__ {{
            displayport = <0x{dcpext:x}>;
        }};
    }};

    fragment@2 {{
        target-path = "{DISPLAY_PATH}";
        __overlay__ {{
            iommus = <0x{disp0_dart:x} 0 0x{dispext0_dart:x} 0>;
        }};
    }};

    fragment@3 {{
        target-path = "{DISPEXT0_DART_PATH}";
        __overlay__ {{ status = "okay"; }};
    }};

    fragment@4 {{
        target-path = "{DCPEXT_DART_PATH}";
        __overlay__ {{ status = "okay"; }};
    }};

    fragment@5 {{
        target-path = "{DCPEXT_MBOX_PATH}";
        __overlay__ {{ status = "okay"; }};
    }};

    fragment@6 {{
        target-path = "{SOC_PATH}";
        __overlay__ {{
            atcphy1_xbar: mux@50304c000 {{
                compatible = "apple,t8103-display-crossbar";
                reg = <0x5 0x0304c000 0x0 0x4000>;
                #mux-control-cells = <1>;
                power-domains = <0x{atc1_power[0]:x}>;
                status = "okay";
            }};
        }};
    }};

    fragment@7 {{
        target-path = "{DCPEXT_PATH}";
        __overlay__ {{
            status = "okay";
            apple,connector-type = "DP";
            apple,dptx-phy = <1>;
            phys = <0x{atcphy1:x} {PHY_TYPE_DP}>;
            phy-names = "dp-phy";
            mux-controls = <&atcphy1_xbar 0>;
            mux-control-names = "dp-xbar";
            mux-index = <0>;
        }};
    }};

    fragment@8 {{
        target-path = "{ATC1_COMMON_PATH}";
        __overlay__ {{ apple,always-on; }};
    }};
}};
"""


def patch_dtb_file(
    input_dtb: pathlib.Path,
    output_dtb: pathlib.Path,
    machine: str = SUPPORTED_MACHINE,
) -> bool:
    """Patch a standalone DTB and return True when it changed."""
    if machine != SUPPORTED_MACHINE:
        raise DcpextDtbError(f"DCPext runtime topology is unsupported for {machine}")
    if dtb_has_dcpext(input_dtb):
        if input_dtb.resolve() != output_dtb.resolve():
            output_dtb.write_bytes(input_dtb.read_bytes())
        return False

    with tempfile.TemporaryDirectory() as directory:
        directory_path = pathlib.Path(directory)
        overlay_dts = directory_path / "apple-dcpext-overlay.dts"
        overlay_dtbo = directory_path / "apple-dcpext-overlay.dtbo"
        overlay_dts.write_text(_overlay_dts(input_dtb))
        _run(
            [
                _tool("dtc"),
                "-@",
                "-I",
                "dts",
                "-O",
                "dtb",
                "-o",
                overlay_dtbo,
                overlay_dts,
            ]
        )
        _run([_tool("fdtoverlay"), "-i", input_dtb, "-o", output_dtb, overlay_dtbo])

    if not dtb_has_dcpext(output_dtb):
        raise DcpextDtbError("patched J293 DTB failed DCPext topology validation")
    return True


def patch_dtb_bytes(
    dtb: bytes,
    machine: str = SUPPORTED_MACHINE,
) -> tuple[bytes, bool]:
    with tempfile.TemporaryDirectory() as directory:
        directory_path = pathlib.Path(directory)
        input_dtb = directory_path / "input.dtb"
        output_dtb = directory_path / "output.dtb"
        input_dtb.write_bytes(dtb)
        changed = patch_dtb_file(input_dtb, output_dtb, machine)
        return output_dtb.read_bytes(), changed


def _payload_dtb_offset(payload: bytes, m1n1_bin: pathlib.Path | None) -> int:
    if m1n1_bin is not None and m1n1_bin.exists():
        offset = len(m1n1_bin.read_bytes())
        if payload[offset : offset + len(FDT_MAGIC)] == FDT_MAGIC:
            return offset
    offset = payload.find(FDT_MAGIC)
    if offset < 0:
        raise DcpextDtbError("payload does not contain an FDT blob")
    return offset


def patch_payload_bytes(
    payload: bytes,
    machine: str = SUPPORTED_MACHINE,
    m1n1_bin: pathlib.Path | None = None,
) -> tuple[bytes, bool]:
    """Patch the DTB embedded in m1n1+DTB+U-Boot payload bytes."""
    offset = _payload_dtb_offset(payload, m1n1_bin)
    if len(payload) < offset + 8:
        raise DcpextDtbError("payload FDT header is truncated")
    dtb_size = int.from_bytes(payload[offset + 4 : offset + 8], "big")
    dtb_end = offset + dtb_size
    if dtb_size <= 0 or dtb_end > len(payload):
        raise DcpextDtbError("payload FDT totalsize is invalid")
    patched_dtb, changed = patch_dtb_bytes(payload[offset:dtb_end], machine)
    return payload[:offset] + patched_dtb + payload[dtb_end:], changed


def patch_payload_file(
    input_payload: pathlib.Path,
    output_payload: pathlib.Path,
    machine: str = SUPPORTED_MACHINE,
    m1n1_bin: pathlib.Path | None = None,
) -> bool:
    patched, changed = patch_payload_bytes(
        input_payload.read_bytes(),
        machine,
        m1n1_bin,
    )
    output_payload.write_bytes(patched)
    return changed


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("patch-dtb", "patch-payload"):
        sub = subparsers.add_parser(command)
        sub.add_argument("--machine", default=SUPPORTED_MACHINE)
        sub.add_argument("--input", type=pathlib.Path, required=True)
        sub.add_argument("--output", type=pathlib.Path, required=True)
        if command == "patch-payload":
            sub.add_argument("--m1n1-bin", type=pathlib.Path)

    args = parser.parse_args()
    try:
        if args.command == "patch-dtb":
            changed = patch_dtb_file(args.input, args.output, args.machine)
        elif args.command == "patch-payload":
            changed = patch_payload_file(
                args.input,
                args.output,
                args.machine,
                args.m1n1_bin,
            )
        else:
            raise AssertionError(args.command)
        action = "patched" if changed else "already-present"
        print(f"{action}: {args.machine} Type-C DCPext topology", file=sys.stderr)
        return 0
    except (DcpextDtbError, subprocess.CalledProcessError) as exc:
        print(f"apple-dcpext-dtb: {exc}", file=sys.stderr)
        if isinstance(exc, subprocess.CalledProcessError) and exc.stderr:
            print(exc.stderr, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(_main())
