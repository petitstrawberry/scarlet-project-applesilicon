#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Patch J293 Type-C DCPext topology into an m1n1 payload DTB.

The stock J293 DTB already contains DCPext, its mailbox, both display DARTs,
and the SIO DART, but keeps DCPext disabled and omits the SIO/DPAudio nodes
needed for DisplayPort audio.  This helper enables the common display
resources, advertises both Type-C ports, and adds the static SIO/DPAudio
topology.  m1n1 then fills the SIO firmware reservations and parameters from
the live ADT before handing the tree to Scarlet.
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

ALIASES_PATH = "/aliases"
DISPLAY_PATH = "/soc/display-subsystem"
DISP0_DART_PATH = "/soc/iommu@231304000"
DISPEXT0_DART_PATH = "/soc/iommu@271304000"
DCPEXT_DART_PATH = "/soc/iommu@27130c000"
DCPEXT_MBOX_PATH = "/soc/mbox@271c08000"
DCPEXT_PATH = "/soc/dcp@271c00000"
SIO_DART_PATH = "/soc/iommu@235004000"
SIO_MBOX_PATH = "/soc/mbox@236408000"
SIO_PATH = "/soc/sio@236400000"
DPAUDIO1_PATH = "/soc/audio-controller@238334000"
DPAUDIO1_ENDPOINT_PATH = DPAUDIO1_PATH + "/ports/port@0/endpoint"
DCPEXT_AUDIO_ENDPOINT_PATH = DCPEXT_PATH + "/ports/port@0/endpoint"
AIC_PATH = "/soc/interrupt-controller@23b100000"
PMGR_PATH = "/soc/power-management@23b700000"
SIO_POWER_PATH = PMGR_PATH + "/power-controller@1c8"
SIO_CPU_POWER_PATH = PMGR_PATH + "/power-controller@1d0"
DPAUDIO1_POWER_PATH = PMGR_PATH + "/power-controller@2f0"
ATCPHY0_PATH = "/soc/phy@383000000"
ATCPHY1_PATH = "/soc/phy@503000000"
TYPEC0_PATH = "/soc/i2c@235010000/usb-pd@38/connector"
TYPEC1_PATH = "/soc/i2c@235010000/usb-pd@3f/connector"
ATC0_COMMON_PATH = "/soc/power-management@23b700000/power-controller@420"
ATC1_COMMON_PATH = "/soc/power-management@23b700000/power-controller@448"

# These properties used to bind DCPext to ATC1 before its driver probe.  That
# prevents the rear port from ever reaching the driver's runtime route
# selection when firmware leaves only ATC0 enabled.
LEGACY_FIXED_ROUTE_PROPERTIES = (
    "phys",
    "phy-names",
    "mux-controls",
    "mux-control-names",
    "mux-index",
)


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
        SIO_DART_PATH,
        AIC_PATH,
        SIO_POWER_PATH,
        SIO_CPU_POWER_PATH,
        DPAUDIO1_POWER_PATH,
        ATCPHY0_PATH,
        ATCPHY1_PATH,
        TYPEC0_PATH,
        TYPEC1_PATH,
        ATC0_COMMON_PATH,
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
        sio_dart = _phandle(dtb, SIO_DART_PATH)
        sio_mbox = _phandle(dtb, SIO_MBOX_PATH)
        sio = _phandle(dtb, SIO_PATH)
        aic = _phandle(dtb, AIC_PATH)
        sio_power = _phandle(dtb, SIO_POWER_PATH)
        sio_cpu_power = _phandle(dtb, SIO_CPU_POWER_PATH)
        dpaudio1_power = _phandle(dtb, DPAUDIO1_POWER_PATH)
        dpaudio1_endpoint = _phandle(dtb, DPAUDIO1_ENDPOINT_PATH)
        dcpext_audio_endpoint = _phandle(dtb, DCPEXT_AUDIO_ENDPOINT_PATH)
    except DcpextDtbError:
        return False

    expected_status = (DISPEXT0_DART_PATH, DCPEXT_DART_PATH, DCPEXT_MBOX_PATH, DCPEXT_PATH)
    return (
        _string_property(dtb, ALIASES_PATH, "dcpext") == DCPEXT_PATH
        and _string_property(dtb, ALIASES_PATH, "sio") == SIO_PATH
        and _cells_property(dtb, TYPEC0_PATH, "displayport") == [dcpext]
        and _cells_property(dtb, TYPEC1_PATH, "displayport") == [dcpext]
        and _cells_property(dtb, DISPLAY_PATH, "iommus")
        == [disp0_dart, 0, dispext0_dart, 0]
        and all(_string_property(dtb, path, "status") == "okay" for path in expected_status)
        and _string_property(dtb, DCPEXT_PATH, "apple,connector-type") == "DP"
        and _cells_property(dtb, DCPEXT_PATH, "apple,dptx-phy") == [1]
        and not any(
            _has_property(dtb, DCPEXT_PATH, name)
            for name in LEGACY_FIXED_ROUTE_PROPERTIES
        )
        and _cells_property(dtb, DCPEXT_PATH, "iommus") == [dcpext_dart, 0]
        and _cells_property(dtb, DCPEXT_PATH, "mboxes") == [dcpext_mbox]
        and _string_property(dtb, SIO_DART_PATH, "status") == "okay"
        and _string_property(dtb, SIO_MBOX_PATH, "compatible")
        == "apple,t8103-asc-mailbox apple,asc-mailbox-v4"
        and _string_property(dtb, SIO_MBOX_PATH, "status") == "okay"
        and _cells_property(dtb, SIO_MBOX_PATH, "reg")
        == [2, 0x36408000, 0, 0x4000]
        and _cells_property(dtb, SIO_MBOX_PATH, "interrupt-parent") == [aic]
        and _cells_property(dtb, SIO_MBOX_PATH, "interrupts")
        == [0, 640, 4, 0, 641, 4, 0, 642, 4, 0, 643, 4]
        and _cells_property(dtb, SIO_MBOX_PATH, "#mbox-cells") == [0]
        and _cells_property(dtb, SIO_MBOX_PATH, "power-domains") == [sio_power]
        and _string_property(dtb, SIO_PATH, "compatible")
        == "apple,t8103-sio apple,sio"
        and _string_property(dtb, SIO_PATH, "status") in ("disabled", "okay")
        and _cells_property(dtb, SIO_PATH, "reg")
        == [2, 0x36400000, 0, 0x8000]
        and _cells_property(dtb, SIO_PATH, "dma-channels") == [128]
        and _cells_property(dtb, SIO_PATH, "#dma-cells") == [1]
        and _cells_property(dtb, SIO_PATH, "mboxes") == [sio_mbox]
        and _cells_property(dtb, SIO_PATH, "iommus") == [sio_dart, 0]
        and _cells_property(dtb, SIO_PATH, "power-domains") == [sio_cpu_power]
        and _cells_property(dtb, SIO_PATH, "resets") == [sio_power]
        and _string_property(dtb, DPAUDIO1_PATH, "compatible")
        == "apple,t8103-dpaudio apple,dpaudio"
        and _string_property(dtb, DPAUDIO1_PATH, "status") == "okay"
        and _cells_property(dtb, DPAUDIO1_PATH, "reg")
        == [2, 0x38334000, 0, 0x4000]
        and _cells_property(dtb, DPAUDIO1_PATH, "dmas") == [sio, 0x66]
        and _string_property(dtb, DPAUDIO1_PATH, "dma-names") == "tx"
        and _cells_property(dtb, DPAUDIO1_PATH, "power-domains")
        == [dpaudio1_power]
        and _cells_property(dtb, DPAUDIO1_PATH, "reset-domains")
        == [dpaudio1_power]
        and _cells_property(dtb, DPAUDIO1_PATH, "resets")
        == [dpaudio1_power]
        and _cells_property(dtb, DPAUDIO1_ENDPOINT_PATH, "remote-endpoint")
        == [dcpext_audio_endpoint]
        and _cells_property(dtb, DCPEXT_AUDIO_ENDPOINT_PATH, "remote-endpoint")
        == [dpaudio1_endpoint]
        and _has_property(dtb, ATC0_COMMON_PATH, "apple,always-on")
        and _has_property(dtb, ATC1_COMMON_PATH, "apple,always-on")
    )


def _overlay_dts(dtb: pathlib.Path) -> str:
    _require_j293_nodes(dtb)
    dcpext = _phandle(dtb, DCPEXT_PATH)
    disp0_dart = _phandle(dtb, DISP0_DART_PATH)
    dispext0_dart = _phandle(dtb, DISPEXT0_DART_PATH)
    sio_dart = _phandle(dtb, SIO_DART_PATH)
    aic = _phandle(dtb, AIC_PATH)
    sio_power = _phandle(dtb, SIO_POWER_PATH)
    sio_cpu_power = _phandle(dtb, SIO_CPU_POWER_PATH)

    return f"""/dts-v1/;
/plugin/;

/ {{
    fragment@0 {{
        target-path = "{ALIASES_PATH}";
        __overlay__ {{
            dcpext = "{DCPEXT_PATH}";
            sio = "{SIO_PATH}";
        }};
    }};

    fragment@1 {{
        target-path = "{TYPEC0_PATH}";
        __overlay__ {{
            displayport = <0x{dcpext:x}>;
        }};
    }};

    fragment@2 {{
        target-path = "{TYPEC1_PATH}";
        __overlay__ {{
            displayport = <0x{dcpext:x}>;
        }};
    }};

    fragment@3 {{
        target-path = "{DISPLAY_PATH}";
        __overlay__ {{
            iommus = <0x{disp0_dart:x} 0 0x{dispext0_dart:x} 0>;
        }};
    }};

    fragment@4 {{
        target-path = "{DISPEXT0_DART_PATH}";
        __overlay__ {{ status = "okay"; }};
    }};

    fragment@5 {{
        target-path = "{DCPEXT_DART_PATH}";
        __overlay__ {{ status = "okay"; }};
    }};

    fragment@6 {{
        target-path = "{DCPEXT_MBOX_PATH}";
        __overlay__ {{ status = "okay"; }};
    }};

    fragment@7 {{
        target-path = "{DCPEXT_PATH}";
        __overlay__ {{
            status = "okay";
            apple,connector-type = "DP";
            apple,dptx-phy = <1>;
        }};
    }};

    fragment@8 {{
        target-path = "{ATC0_COMMON_PATH}";
        __overlay__ {{ apple,always-on; }};
    }};

    fragment@9 {{
        target-path = "{ATC1_COMMON_PATH}";
        __overlay__ {{ apple,always-on; }};
    }};

    fragment@10 {{
        target-path = "{SIO_DART_PATH}";
        __overlay__ {{ status = "okay"; }};
    }};

    fragment@11 {{
        target-path = "{DPAUDIO1_POWER_PATH}";
        dpaudio1_power: __overlay__ {{ }};
    }};

    fragment@12 {{
        target-path = "/soc";
        __overlay__ {{
            sio_mbox: mbox@236408000 {{
                compatible = "apple,t8103-asc-mailbox", "apple,asc-mailbox-v4";
                reg = <0x2 0x36408000 0x0 0x4000>;
                interrupt-parent = <0x{aic:x}>;
                interrupts = <0 640 4>, <0 641 4>, <0 642 4>, <0 643 4>;
                interrupt-names = "send-empty", "send-not-empty",
                                  "recv-empty", "recv-not-empty";
                #mbox-cells = <0>;
                power-domains = <0x{sio_power:x}>;
                status = "okay";
            }};

            sio: sio@236400000 {{
                compatible = "apple,t8103-sio", "apple,sio";
                reg = <0x2 0x36400000 0x0 0x8000>;
                dma-channels = <128>;
                #dma-cells = <1>;
                mboxes = <&sio_mbox>;
                iommus = <0x{sio_dart:x} 0>;
                power-domains = <0x{sio_cpu_power:x}>;
                resets = <0x{sio_power:x}>;
                status = "disabled";
            }};

            dpaudio1: audio-controller@238334000 {{
                compatible = "apple,t8103-dpaudio", "apple,dpaudio";
                reg = <0x2 0x38334000 0x0 0x4000>;
                dmas = <&sio 0x66>;
                dma-names = "tx";
                power-domains = <&dpaudio1_power>;
                reset-domains = <&dpaudio1_power>;
                resets = <&dpaudio1_power>;
                status = "okay";

                ports {{
                    #address-cells = <1>;
                    #size-cells = <0>;
                    port@0 {{
                        reg = <0>;
                        dpaudio1_dcp: endpoint {{
                            remote-endpoint = <&dcpext_audio>;
                        }};
                    }};
                }};
            }};
        }};
    }};

    fragment@13 {{
        target-path = "{DCPEXT_PATH}";
        __overlay__ {{
            #address-cells = <2>;
            #size-cells = <2>;
            ports {{
                #address-cells = <1>;
                #size-cells = <0>;
                port@0 {{
                    reg = <0>;
                    dcpext_audio: endpoint {{
                        remote-endpoint = <&dpaudio1_dcp>;
                    }};
                }};
            }};
        }};
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

        # Migrate payloads previously patched with the ATC1-only topology.
        # Device-tree overlays cannot reliably delete base properties, so use
        # fdtput after applying the additive overlay.
        for name in LEGACY_FIXED_ROUTE_PROPERTIES:
            if _has_property(output_dtb, DCPEXT_PATH, name):
                _run([_tool("fdtput"), "-d", output_dtb, DCPEXT_PATH, name])

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
