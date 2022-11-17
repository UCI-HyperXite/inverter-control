from typing import Iterable

from pdm import run_lengths, sPWM
from pio import assemble_program, emulate_pio

INVERTER_CODE = "inverter.pio"
T_SWITCH = 3  # number of instructions needed to alternate output


def pad_durations(s: Iterable[int]) -> list[int]:
    """Adjusts PDM durations to account for switching time in PIO"""
    return [max(0, T_SWITCH * (n - 1)) for n in s]


def invert(s: Iterable[int]) -> list[int]:
    """Converts bit stream into alternating values"""
    return [1 if v else -1 for v in s]


def run_inverter(N: int) -> list[int]:
    """Runs the inverter PIO code to simulate a sine wave.
    The length of the output will be scaled by the switching time."""
    pdm = sPWM(N)
    d = pad_durations(run_lengths(pdm))

    asm = assemble_program(INVERTER_CODE)
    duration = sum(d) + T_SWITCH * len(d)
    output = emulate_pio(asm, duration, d)

    return invert(output)
