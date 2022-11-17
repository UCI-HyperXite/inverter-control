from collections import deque
from typing import Iterator, Reversible

from adafruit_pioasm import assemble
from pioemu import State, emulate


def assemble_program(program_file: str):
    """Assembles a PIO assembly code file"""
    with open(program_file) as program:
        asm: list[int] = assemble(program.read())
        return asm


def emulate_pio(asm, duration: int, fifo: Reversible[int]) -> Iterator[int]:
    """Emulates a PIO assembly program for the given duration with the given FIFO input"""
    generator = emulate(
        asm,
        stop_when=lambda opcodes, state: state.clock >= duration,
        initial_state=State(transmit_fifo=deque(reversed(fifo))),
        side_set_base=0,
        side_set_count=1,
    )
    for _, state in generator:
        yield state.pin_values
