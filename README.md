This fork of Webots is being developed as the platform for K12-level STEM school competitions, specifically space-related ones, even more specifically - https://junior.ntcontest.ru/ .  

Note that this is currently my private project, uploaded to github mostly to sync between my several computers, so please don't expect to find here something finished and tested, it's not.  The only useful thing in this repo at the moment is the translation file!

Here is what's going to be added/changed (listed more or less in the order of priorities):

- Full translation to Russian  (mostly done, but need to keep up with additions in cyberbotics/webots)

** code in progress, not ready, won't even compile yet. 

- Adding thrusters, as PROTOs (based on the Propeller) or a new device type

** not started yet **

- Adding a fuel system (something KSP-style): thrusters spend fuel, changing rocket mass, stop when no fuel left

- Some form of integration with Stepik.org learning platform (based on "robot windows": get a random token from Stepik, measure results in the simulation, producing a score, as the answer encrypt score using the token, let the student copy the answer to Stepik) 

- Adding gravitating bodies (Solids as gravity sources)

- Adding PROTOs for various space-related parts and pieces

- Adding robot windows for monitoring state of a satellite, 

- Possibly integration with WokWi or similar Arduino simulator

- Adding a library of PROTOs for common AliExpress hardware (motors, sensor modules and such)

-----------------------

# Webots: open-source robot simulator

[![Webots](https://img.shields.io/github/v/release/cyberbotics/webots)](https://github.com/cyberbotics/webots/releases/latest)
[![Software License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)
[![User Guide](https://img.shields.io/badge/doc-guide-blue)](https://cyberbotics.com/doc/reference/index)
[![Reference Manual](https://img.shields.io/badge/doc-reference-blue.svg)](https://cyberbotics.com/doc/reference/index)<br>
[![Stars](https://img.shields.io/github/stars/cyberbotics/webots)](https://github.com/cyberbotics/webots/stargazers)
[![Downloads](https://img.shields.io/github/downloads/cyberbotics/webots/total?color=blue)](https://hanadigital.github.io/grev/?user=cyberbotics&repo=webots)
[![Contributions](https://img.shields.io/github/commit-activity/m/cyberbotics/webots.svg)](https://github.com/cyberbotics/webots/graphs/commit-activity)
[![Contributors](https://img.shields.io/github/contributors/cyberbotics/webots?color=blue)](https://github.com/cyberbotics/webots/graphs/contributors)
[![GitHub Discussions](https://img.shields.io/github/discussions/cyberbotics/webots)](https://github.com/cyberbotics/webots/discussions)
[![Chat](https://img.shields.io/discord/565154702715518986?color=blue)](https://discordapp.com/invite/nTWbN9m)


![Webots Screenshot](docs/guide/images/main_window.png?raw=true "Webots Screenshot")

Webots provides a complete development environment to model, program and simulate robots, vehicles and mechanical systems. See the [Webots introduction video](https://www.youtube.com/watch?v=O7U3sX_ubGc).

### Download

Get pre-compiled binaries for the [latest release](https://github.com/cyberbotics/webots/releases/latest), as well as [older releases and nightly builds](https://github.com/cyberbotics/webots/releases).

Check out installation instructions:

[![Linux](https://img.shields.io/badge/Linux-0f80c0?logo=linux&logoColor=white)](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)
[![Windows](https://img.shields.io/badge/Windows-0f80c0?logo=windows&logoColor=white)](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-windows)
[![macOS](https://img.shields.io/badge/macOS-0f80c0?logo=apple&logoColor=white)](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-macos)

### Build from Source

If you prefer to [compile Webots from source](https://github.com/cyberbotics/webots/wiki), read the [contributing guidelines](CONTRIBUTING.md).

### Continuous Integration Nightly Tests

[![master branch](https://img.shields.io/badge/branch-master-blue)](https://github.com/cyberbotics/webots/tree/master)
[![Linux build (master)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_linux.yml/badge.svg?event=schedule)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_linux.yml?query=event%3Aschedule)
[![Windows build (master)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_windows.yml/badge.svg?event=schedule)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_windows.yml?query=event%3Aschedule)
[![macOS build (master)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_mac.yml/badge.svg?event=schedule&label=macOS)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_mac.yml?query=event%3Aschedule)<br>
[![develop branch](https://img.shields.io/badge/branch-develop-blue)](https://github.com/cyberbotics/webots/tree/develop)
[![Linux build (develop)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_linux_develop.yml/badge.svg?event=schedule)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_linux_develop.yml?query=event%3Aschedule)
[![Windows build (develop)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_windows.yml/badge.svg?event=schedule)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_windows_develop.yml?query=event%3Aschedule)
[![macOS build (develop)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_mac.yml/badge.svg?event=schedule)](https://github.com/cyberbotics/webots/actions/workflows/test_suite_mac_develop.yml?query=event%3Aschedule)

### About us

Webots was originally designed at [EPFL](https://epfl.ch) in 1996 and then further developed and commercialized by [Cyberbotics](https://cyberbotics.com) since 1998. In December 2018, Webots was open sourced. Since then, [Cyberbotics](https://cyberbotics.com) continues to develop Webots thanks to paid customer support, training, consulting for industry and academic research projects.

[Contact us](mailto:info@cyberbotics.com) to discuss your custom robot simulation projects.
