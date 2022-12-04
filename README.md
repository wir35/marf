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
    6.  Power back on.
5.  The old cal data is now wrong, so run the calibration procedure below.

### Module Calibration

Due to some questionable hardware design choices, the module _absolutely requires_ calibration
for correct behavior of panel controls and external inputs. Use the following procedure.

1.  Hold Stage Address 1 Advance down during start up.
2.  Note leds cycling.
3.  Turn all pots to maximum. The slider positions do not matter.
4.  Apply a calibrated 10v source to all 4 external inputs.
5.  Select pulse 2 up if you need to swap the pulse leds. Led will move to pulse 2.
6.  Select pulse 1 up to swap it back to normal.
7.  Press Stage Address 2 Advance down to save calibration.
8.  It will ERASE the entire eprom, including saved programs.

(Some hardware revisions had the pulse leds switched by accident. 
You only need this part of the procedure if you are affected by this bug.)

### Recommended Hardware Modifications

The module can also be improved by making several hardware modifications.
Visit [Dave Brown's Page](https://modularsynthesis.com/roman/buchla248/248_mods.htm)
for a comprehensive description of the recommended changes.

## Version History

This module's firmware has had a wild ride through a few generations.

### v2.66

The project was extensively rewritten by [maxl0rd](https://github.com/maxl0rd)
to more closely match the functionality of the original module,
and to resolve many performance problems.

See the [RELEASE_NOTES](https://github.com/wir35/marf/blob/v2.66/RELEASE_NOTES.txt) 
file for a description of both the user-visible and internal changes.

Expander modules provided by SAModular have been tested with this firmware
and operate correctly.

### v2.5

The project was converted from the Keil toolchain to STM32Cube by 
[Steven Barsky](https://github.com/stevenbarsky) and a good number of bugs were fixed,
including expander support.

Most new builds are currently programmed with this release.

### v1.0 and v2.0 Branches

These branches are the verbatim, original code drop from roman_f.

Released by roman_f on Oct 8, 2019.
See the [MuffWiggler Thread](https://www.muffwiggler.com/forum/viewtopic.php?t=222687).

Neither of these branches is verified, or known to work.
The hex firmware images checked in are not identical to the published firmware images
on the [Electronic Music Store build page](https://electricmusicstore.com/blogs/build/115318789-multiple-arbitrary-function-generator-model-248).

I do not recommend attempting to use these images for new builds.
