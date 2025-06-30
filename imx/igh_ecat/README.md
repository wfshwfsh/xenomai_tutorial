This is the README file of the IgH EtherCAT Master.

Contents:

[[_TOC_]]

# General Information

This is an open-source EtherCAT master implementation for Linux 2.6 or newer.

See the [features file](FEATURES.md) for a list of features. For more
information, see https://etherlab.org/ethercat.

or contact

>>>
Dipl.-Ing. (FH) Florian Pose <fp@igh.de>
Ingenieurgemeinschaft IgH
Nordsternstraße 66
D-45329 Essen
http://igh.de
>>>

# Documentation

## Handbook

The PDF documentation is generated via LaTeX and can be build with the
following steps:

```bash
cd documentation
make
```

The PDF is automatically held up-to-date and can be [downloaded from
GitLab](https://gitlab.com/etherlab.org/ethercat/-/jobs/artifacts/stable-1.5/raw/pdf/ethercat_doc.pdf?job=pdf).

## Doxygen

To generate the Doxygen documentation, the following commands can be used.
Therefore, the configure script must have run (see the [install
file](INSTALL.md)).

```bash
git submodule update --init
make doc
```

An up-to-date Doxygen output can be found on
[docs.etherlab.org](https://docs.etherlab.org/ethercat/1.6/doxygen/index.html).

# Requirements

## Software requirements

Configured sources for the Linux 2.6 (or newer) kernel are required to build
the EtherCAT master.

## Hardware requirements

A table of supported hardware can be found at:
https://docs.etherlab.org/ethercat/1.6/doxygen/devicedrivers.html

# Building and installing

See the [install file](INSTALL.md).

# Dry-run and Field Simulation

A limited set of the userspace API is available in `libfakeethercat`,
a library which can be used to run an userspace application
without an EtherCAT master or with emulated EtherCAT slaves.
Please find some details [here](fake_lib/README.md).

# Realtime and Tuning

Realtime patches for the Linux kernel are supported, but not required. The
realtime processing has to be done by the calling module (see API
documentation). The EtherCAT master code itself is passive (except for the
idle mode and EoE).

To avoid frame timeouts, deactivating DMA access for hard drives is
recommended (`hdparm -d0 <DEV>`).

# License

Copyright (C) 2006-2023  Florian Pose, Ingenieurgemeinschaft IgH

This file is part of the IgH EtherCAT Master.

The IgH EtherCAT Master is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License version 2, as
published by the Free Software Foundation.

The IgH EtherCAT Master is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
the IgH EtherCAT Master; if not, write to the Free Software Foundation, Inc.,
51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


# Coding Style

Developers shall use the coding style rules in the [coding style
file](CodingStyle.md).
