#!/usr/bin/env python3
"""Unit tests for deploy_m1n1_usb helpers that do not require hardware."""

import importlib.util
import pathlib
import subprocess
import sys
import tempfile
import types
import unittest
from types import SimpleNamespace
from unittest import mock


MODULE_PATH = pathlib.Path(__file__).with_name("deploy_m1n1_usb.py")
SPEC = importlib.util.spec_from_file_location("deploy_m1n1_usb", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ChainloadFailureTests(unittest.TestCase):
    def test_uboot_build_refreshes_m1n1_before_uboot(self):
        build_script = MODULE_PATH.parents[1].joinpath("build-uboot.sh").read_text()
        m1n1_build = (
            'make -C "$M1N1" TOOLCHAIN= LLDDIR= BUILDSTD=1 build/m1n1.bin'
        )
        uboot_build = 'docker run --rm -v "$UBOOT":/work'

        self.assertIn(
            'M1N1_BASE_COMMIT="e132477af421247dbdad654e527ff230c0abfb71"',
            build_script,
        )
        self.assertIn(
            'if [ ! -f "$M1N1/rust/vendor/rust-fatfs/Cargo.toml" ]',
            build_script,
        )
        self.assertIn('M1N1_VERSION_TAG="$m1n1_version"', build_script)
        self.assertIn('"$PAYLOAD_BUILDER" compose', build_script)
        self.assertLess(build_script.index(m1n1_build), build_script.index(uboot_build))

    def test_payload_auto_skips_current_artifact(self):
        args = SimpleNamespace(
            machine="j293",
            m1n1=MODULE.DEFAULT_M1N1.resolve(),
            payload=MODULE.default_payload_path("j293").resolve(),
            payload_build="auto",
        )

        with (
            mock.patch.object(
                MODULE.apple_boot_payload,
                "check_fresh",
                return_value=(True, "payload is current"),
            ),
            mock.patch.object(MODULE, "run_checked") as run_checked,
        ):
            MODULE.ensure_boot_payload(args)

        run_checked.assert_not_called()

    def test_payload_auto_rebuilds_stale_artifact(self):
        args = SimpleNamespace(
            machine="j293",
            m1n1=MODULE.DEFAULT_M1N1.resolve(),
            payload=MODULE.default_payload_path("j293").resolve(),
            payload_build="auto",
        )

        with (
            mock.patch.object(
                MODULE.apple_boot_payload,
                "check_fresh",
                side_effect=((False, "payload inputs changed"), (True, "current")),
            ),
            mock.patch.object(MODULE, "run_checked") as run_checked,
        ):
            MODULE.ensure_boot_payload(args)

        run_checked.assert_called_once_with(
            [MODULE.BUILD_UBOOT, "j293"], cwd=MODULE.PROJECT_DIR
        )

    def test_payload_always_rebuilds_current_artifact(self):
        args = SimpleNamespace(
            machine="j293",
            m1n1=MODULE.DEFAULT_M1N1.resolve(),
            payload=MODULE.default_payload_path("j293").resolve(),
            payload_build="always",
        )

        with (
            mock.patch.object(
                MODULE.apple_boot_payload,
                "check_fresh",
                side_effect=((True, "payload is current"), (True, "current")),
            ),
            mock.patch.object(MODULE, "run_checked") as run_checked,
        ):
            MODULE.ensure_boot_payload(args)

        run_checked.assert_called_once_with(
            [MODULE.BUILD_UBOOT, "j293"], cwd=MODULE.PROJECT_DIR
        )

    def test_payload_auto_preserves_custom_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            directory_path = pathlib.Path(directory)
            m1n1 = directory_path / "m1n1.bin"
            payload = directory_path / "boot.bin"
            m1n1.write_bytes(b"m1n1")
            payload.write_bytes(b"payload")
            args = SimpleNamespace(
                machine="j293",
                m1n1=m1n1,
                payload=payload,
                payload_build="auto",
            )

            with (
                mock.patch.object(
                    MODULE.apple_boot_payload, "check_fresh"
                ) as check_fresh,
                mock.patch.object(MODULE, "run_checked") as run_checked,
            ):
                MODULE.ensure_boot_payload(args)

            check_fresh.assert_not_called()
            run_checked.assert_not_called()

    def test_payload_never_rejects_stale_artifact(self):
        args = SimpleNamespace(
            machine="j293",
            m1n1=MODULE.DEFAULT_M1N1.resolve(),
            payload=MODULE.default_payload_path("j293").resolve(),
            payload_build="never",
        )

        with mock.patch.object(
            MODULE.apple_boot_payload,
            "check_fresh",
            return_value=(False, "payload inputs changed"),
        ):
            with self.assertRaisesRegex(RuntimeError, "payload inputs changed"):
                MODULE.ensure_boot_payload(args)

    def test_project_runner_disables_rebuild(self):
        runner = MODULE_PATH.with_name("run.sh").read_text()

        self.assertIn('deploy_m1n1_usb.py" --no-build "$@"', runner)

    def test_missing_tool_fails_environment_check(self):
        with mock.patch.object(MODULE.shutil, "which", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "missing command.*ld.lld"):
                MODULE.ensure_commands(("ld.lld",))

    def test_chainload_waits_for_proxy_device_before_starting(self):
        with tempfile.TemporaryDirectory() as directory:
            payload = pathlib.Path(directory) / "m1n1.bin"
            payload.write_bytes(b"m1n1")
            args = SimpleNamespace(
                proxy_device="/dev/cu.test-proxy",
                m1n1=payload,
                connect_timeout=1.0,
            )
            result = subprocess.CompletedProcess(["chainload.py"], 0, "", "")

            with (
                mock.patch.object(MODULE, "wait_for_device") as wait_for_device,
                mock.patch.object(MODULE, "run_checked_capture", return_value=result),
            ):
                MODULE.chainload(args)

            wait_for_device.assert_called_once()
            self.assertEqual(wait_for_device.call_args.args[0], args.proxy_device)

    def test_serial_connection_failure_is_retryable(self):
        error = subprocess.CalledProcessError(
            1,
            ["chainload.py"],
            stderr="serial.serialutil.SerialException: device not ready",
        )

        self.assertTrue(MODULE.chainload_failure_is_retryable(error))

    def test_missing_module_is_not_retryable(self):
        error = subprocess.CalledProcessError(
            1,
            ["chainload.py"],
            stderr="ModuleNotFoundError: No module named 'construct'",
        )

        self.assertFalse(MODULE.chainload_failure_is_retryable(error))

    def test_missing_file_is_not_retryable(self):
        error = subprocess.CalledProcessError(
            1,
            ["chainload.py"],
            stderr="python3: can't open file 'chainload.py'",
        )

        self.assertFalse(MODULE.chainload_failure_is_retryable(error))

    def test_missing_tool_is_not_retryable(self):
        error = subprocess.CalledProcessError(
            1,
            ["chainload.py"],
            stderr="Command 'ld.lld -maarch64elf' returned non-zero exit status 127",
        )

        self.assertFalse(MODULE.chainload_failure_is_retryable(error))

    def test_proxy_timeout_is_retryable(self):
        error = subprocess.CalledProcessError(
            1,
            ["chainload.py"],
            stderr="m1n1.proxy.UartTimeout: proxy request timed out",
        )

        self.assertTrue(MODULE.chainload_failure_is_retryable(error))

    def test_dcpext_runtime_patch_uses_selected_payload_machine(self):
        patcher = types.ModuleType("apple_dcpext_dtb")
        patcher.patch_payload_bytes = mock.MagicMock(
            return_value=(b"patched", True)
        )
        args = SimpleNamespace(
            dcpext_dtb_patch="require",
            machine="j293",
            m1n1=pathlib.Path("m1n1.bin"),
        )

        with mock.patch.dict(sys.modules, {"apple_dcpext_dtb": patcher}):
            result = MODULE.patch_dcpext_guest_payload(args, b"payload")

        self.assertEqual(result, b"patched")
        patcher.patch_payload_bytes.assert_called_once_with(
            b"payload",
            machine="j293",
            m1n1_bin=args.m1n1,
        )

    def test_hv_mode_loads_selected_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            directory_path = pathlib.Path(directory)
            image = directory_path / "scarlet.img"
            payload = directory_path / "boot.bin"
            image.write_bytes(b"image")
            payload.write_bytes(b"payload")
            args = SimpleNamespace(
                image=image,
                payload=payload,
                image_map_size=0x1000,
                image_addr=0x900000000,
                entry_point=0x800000000,
                connect_timeout=1.0,
            )

            iface = mock.MagicMock()
            proxy = mock.MagicMock()
            proxy_utils = SimpleNamespace(
                ba=SimpleNamespace(phys_base=0x800000000, mem_size=0x200000000)
            )
            hv = mock.MagicMock()
            hv.adt = None
            hv.ram_base = 0x800000000
            proxyutils_module = types.ModuleType("m1n1.proxyutils")
            proxyutils_module.ProxyUtils = lambda _proxy, heap_size: proxy_utils
            hv_module = types.ModuleType("m1n1.hv")
            hv_module.HV = lambda _iface, _proxy, _utils: hv
            pmu = mock.MagicMock()
            pmu_module = types.ModuleType("m1n1.hw.pmu")
            pmu_module.PMU = lambda _proxy_utils: pmu

            with (
                mock.patch.dict(
                    sys.modules,
                    {
                        "m1n1.proxyutils": proxyutils_module,
                        "m1n1.hv": hv_module,
                        "m1n1.hw.pmu": pmu_module,
                    },
                ),
                mock.patch.object(MODULE, "open_proxy", return_value=(iface, proxy)),
                mock.patch.object(
                    MODULE,
                    "patch_dcpext_guest_payload",
                    side_effect=lambda _args, data: data + b"-dcpext",
                ),
                mock.patch.object(
                    MODULE,
                    "patch_avd_guest_payload",
                    side_effect=lambda _args, _adt, data: data + b"-avd",
                ),
                mock.patch.object(MODULE, "enable_avd_pmgr"),
            ):
                MODULE.start_guest(args)

            hv.load_raw.assert_called_once_with(
                b"payload-dcpext-avd",
                args.entry_point,
            )
            iface.writemem.assert_called_once_with(args.image_addr, b"image", True)
            hv.start.assert_called_once_with()

    def test_bare_chainload_does_not_retry_tool_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            directory_path = pathlib.Path(directory)
            image = directory_path / "scarlet.img"
            payload = directory_path / "boot.bin"
            image.write_bytes(b"image")
            payload.write_bytes(b"payload")
            args = SimpleNamespace(
                image=image,
                payload=payload,
                image_map_size=0x1000,
                image_addr=0x900000000,
                proxy_device="/dev/cu.test-proxy",
                connect_timeout=1.0,
                dcpext_dtb_patch="off",
                avd_dtb_patch="off",
                avd_pmgr="off",
            )

            iface = mock.MagicMock()
            proxy = mock.MagicMock()
            proxy_utils = SimpleNamespace(
                adt=None,
                ba=SimpleNamespace(phys_base=0x800000000, mem_size=0x200000000),
            )
            proxyutils_module = types.ModuleType("m1n1.proxyutils")
            proxyutils_module.ProxyUtils = lambda _proxy, heap_size: proxy_utils
            pmu_module = types.ModuleType("m1n1.hw.pmu")
            pmu_module.PMU = lambda _proxy_utils: mock.MagicMock()
            tool_error = subprocess.CalledProcessError(
                1,
                ["chainload.py"],
                stderr="Command 'ld.lld -maarch64elf' returned non-zero exit status 127",
            )

            with (
                mock.patch.dict(
                    sys.modules,
                    {
                        "m1n1.proxyutils": proxyutils_module,
                        "m1n1.hw.pmu": pmu_module,
                    },
                ),
                mock.patch.object(MODULE, "open_proxy", return_value=(iface, proxy)),
                mock.patch.object(MODULE, "wait_for_device"),
                mock.patch.object(MODULE.time, "sleep"),
                mock.patch.object(
                    MODULE,
                    "run_checked_capture",
                    side_effect=tool_error,
                ) as run_chainload,
            ):
                with self.assertRaisesRegex(RuntimeError, "ld.lld"):
                    MODULE.start_guest_bare(args)

            run_chainload.assert_called_once()


if __name__ == "__main__":
    unittest.main()
