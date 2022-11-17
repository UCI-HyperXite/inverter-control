from math import pi, sin
from typing import Callable, Iterable, Iterator


def pdm(x: list[float]) -> Iterator[int]:
    """Converts the given input to a pulse-density modulated signal based on cumulative errors"""
    qe = 0.0
    for s in x:
        qe = qe + s
        y = 1 if qe > 0 else -1
        qe -= y
        yield y


def sample(sig: Callable[[float], float], T: float, N: int) -> list[float]:
    """Samples the given signal function from 0 to T with N samples"""
    return [sig(n * T / N) for n in range(N + 1)]


def sPWM(N: int) -> list[int]:
    """Returns a PDM signal of a sine wave using N samples"""
    return list(pdm(sample(sin, 2 * pi, N)))


def run_lengths(signal: Iterable[int]) -> Iterator[int]:
    """Converts a signal of repeating values into the lengths of each repeated segment"""
    sig = iter(signal)
    last = next(sig)
    c = 1
    for v in sig:
        if last != v:
            yield c
            c = 0
        c += 1
        last = v
    yield c
