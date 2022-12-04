# Multiple Arbitrary Function Generator

This repo contains source code for the Buchla 248r module, originally produced 
by the Electric Music Store and subsequently improved by SA Modular.


## Installation

1.  Obtain an ST-Link STM32 programmer (v2 recommended, available on Amazon for less than $20).
2.  Download [STM32CubeProg](https://www.st.com/en/development-tools/stm32cubeprog.html) from [www.st.com](https://www.st.com).
    It is available for most platforms.
3.  Download and unzip the latest firmware file [marf-v2-66-2022-09-05.hex.zip](https://github.com/wir35/marf/releases/download/marf-v2.66-2022-09-05/marf-v2-66-2022-09-05.hex.zip)
4.  Flash the new firmware.
    1.  Power down the case.
    2.  Remove the module, leaving the power cable connected, and attach the S- Link ribbon cable.
    3.  Power on.
    4.  Program the firmware.
    5.  Power off, disconnect the ST-Link and reseat the module.
    6.  Power back on and enjoy.


## Version History

This module's firmware has had a wild ride through a few generations.

### v2.66

The project was extensively rewritten by [maxl0rd](https://github.com/maxl0rd)
to more closely match the functionality of the original module,
and to resolve many performance problems.

See the [RELEASE_NOTES](https://github.com/wir35/marf/blob/v2.66/RELEASE_NOTES.txt) 
file for a description of both the user-visible and internal changes.

### v2.5

The project was converted from the Keil toolchain to STM32Cube by 
[Steven Barsky](https://github.com/stevenbarsky) and a good number of bugs were fixed.

Most new builds are currently programmed with this release.

### v1.0 and v2.0 Branches

These branches are the verbatim, original code drop from roman_f.

Released by roman_f on Oct 8, 2019.
See the [MuffWiggler Thread](https://www.muffwiggler.com/forum/viewtopic.php?t=222687).

Neither of these branches is verified, or known to work.
The hex firmware images checked in are not identical to the published firmware images
on the [Electronic Music Store build page](https://electricmusicstore.com/blogs/build/115318789-multiple-arbitrary-function-generator-model-248).

I do not recommend attempting to use these images for new builds.
