# Apple Silicon Source Attribution

Scarlet's Apple-specific drivers are Rust implementations for Scarlet's own
driver, memory-management, interrupt, and remote-processor interfaces. Public
Asahi Linux and m1n1 implementations were used to understand hardware register
layouts, firmware protocols, and required sequencing where noted below. A
provenance reference does not by itself mean that a local file is a verbatim
copy of the named upstream source.

The repository's original code remains licensed as stated in [LICENSE](LICENSE).
Git submodules retain their own licenses and notices. This document records
source provenance; it does not replace any upstream license text.

## Review checkpoints

The mappings below were reviewed against these immutable upstream revisions:

- Linux: `1590cf0329716306e948a8fc29f1d3ee87d3989f`
- Asahi Linux: `030248d39b401c94695c9f7df2fed630d35120cd`
- m1n1: `a35e52dba52b1fd0b54dcfec316063362816bdcf`
- Asahi U-Boot: `8aa706b2daa49b64102e44067d8514de8a26dc42`

These are provenance-review checkpoints, not claims that every local line was
derived from those exact revisions.

## Linux and Asahi Linux references

| Scarlet area | Upstream reference | Upstream license and verified notice |
| --- | --- | --- |
| AIC | Linux `drivers/irqchip/irq-apple-aic.c` | GPL-2.0-or-later; `Copyright The Asahi Linux Contributors` |
| DART | Linux `drivers/iommu/apple-dart.c` | GPL-2.0-only; `Copyright (C) 2021 The Asahi Linux Contributors` |
| PMGR | Linux `drivers/pmdomain/apple/pmgr-pwrstate.c` | GPL-2.0-only OR MIT; `Copyright The Asahi Linux Contributors` |
| SPMI and SPMI NVMEM | Linux `drivers/spmi/spmi-apple-controller.c`, `drivers/nvmem/apple-spmi-nvmem.c` | GPL-2.0 / GPL-2.0-only OR MIT; `Copyright The Asahi Linux Contributors` |
| RTKit | Linux `drivers/soc/apple/rtkit.c` | GPL-2.0-only OR MIT; `Copyright (C) The Asahi Linux Contributors` |
| PCIe | Linux `drivers/pci/controller/pcie-apple.c` | GPL-2.0; the source header credits Alyssa Rosenzweig, Google LLC, Corellium LLC, and Mark Kettenis |
| ATC PHY | Linux `drivers/phy/apple/atc.c` | GPL-2.0 OR BSD-2-Clause; `Copyright (C) The Asahi Linux Contributors` |
| DWC3 glue | Linux `drivers/usb/dwc3/dwc3-apple.c` | GPL-2.0; `Copyright (C) The Asahi Linux Contributors` |
| SPI | Linux `drivers/spi/spi-apple.c` | GPL-2.0; `Copyright The Asahi Linux Contributors` |
| MCA | Linux `sound/soc/apple/mca.c` | GPL-2.0-only; `Copyright (C) The Asahi Linux Contributors` |
| ADMAC | Linux `drivers/dma/apple-admac.c` | GPL-2.0-only; `Copyright (C) The Asahi Linux Contributors` |
| SIO DMA | Asahi Linux `drivers/dma/apple-sio.c` | GPL-2.0-only OR MIT; retain the source-specific notice when reusing substantial code |
| NCO | Linux `drivers/clk/clk-apple-nco.c` | GPL-2.0-only OR MIT; `Copyright (C) The Asahi Linux Contributors` |
| CPU frequency | Linux `drivers/cpufreq/apple-soc-cpufreq.c` | GPL-2.0-only; `Copyright The Asahi Linux Contributors` |
| Watchdog | Linux `drivers/watchdog/apple_wdt.c` | GPL-2.0-only OR MIT; `Copyright (C) The Asahi Linux Contributors` |
| eFuse | Linux `drivers/nvmem/apple-efuses.c` | GPL-2.0-only; `Copyright (C) The Asahi Linux Contributors` |
| AFK | Asahi Linux `drivers/gpu/drm/apple/afk.c` | GPL-2.0-only OR MIT; `Copyright 2022 Sven Peter <sven@svenpeter.dev>` |
| EPIC | Asahi Linux `drivers/gpu/drm/apple/systemep.c`, `drivers/gpu/drm/apple/epic/dpavservep.c` | GPL-2.0-only OR MIT; retain the source-specific notices in those files when reusing substantial code |
| DCP, external display, and DisplayPort audio | Asahi Linux `drivers/gpu/drm/apple/dcp.c`, `audio.c`, `av.c`, `dptxep.c`, `ibootep.c`, `parser.c` | GPL-2.0-only OR MIT; retain the source-specific notices in those files when reusing substantial code |

## m1n1 references

m1n1 is MIT licensed and carries the repository-level notice
`Copyright The Asahi Linux Contributors`. Relevant source references are:

- DART: `src/dart.c`, `proxyclient/m1n1/hw/dart.py`
- PMGR: `src/pmgr.c`, `proxyclient/m1n1/hw/pmgr.py`
- ASC and RTKit: `src/asc.c`, `src/rtkit.c`,
  `proxyclient/m1n1/hw/asc.py`, `proxyclient/m1n1/fw/asc/base.py`
- SMC and UART: `src/smc.c`, `src/uart.c`
- AFK and EPIC: `src/afk.c`, `proxyclient/m1n1/fw/afk/epic.py`
- DCP: `src/dcp.c`, `proxyclient/m1n1/fw/dcp/manager.py`
- AVD: `proxyclient/m1n1/fw/avd/decoder.py`

The vendored m1n1 submodule includes its complete `LICENSE` and
`3rdparty_licenses/` directory. Those notices apply to the submodule itself.

## U-Boot

The U-Boot submodule and the Apple-specific U-Boot patch set are distributed
under U-Boot's applicable GPL-2.0-or-later terms. Relevant Apple support
references include `drivers/iommu/apple_dart.c`,
`drivers/power/domain/apple-pmgr.c`, `drivers/mailbox/apple-mbox.c`, and
`arch/arm/mach-apple/rtkit.c` at the U-Boot review checkpoint above. The
submodule's own license files and source headers remain authoritative.

## Deliberately unattributed drivers

Apple-specific hardware alone is not enough evidence of source derivation.
Drivers without a verified source counterpart, including the current Apple I2C,
pinctrl, MSI, CD321x, and several codec implementations, are not attributed to
a particular upstream file here. They should be added only after a concrete
source relationship is established.
