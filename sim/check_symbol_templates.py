#!/usr/bin/env python3
"""Static separability check for symbol_template_rom.v.

Exact templates must always be accepted as their own class. Every possible
single-cell perturbation may either remain correct or be safely rejected, but
must never be accepted as another class.
"""

from pathlib import Path
import re
import sys

MAX_DISTANCE = 12
MIN_MARGIN = 2
ROM = (Path(__file__).resolve().parents[1] /
       "Ti60F225_SC1336_HDMI_Display/src/recognition/symbol_template_rom.v")


def load_templates(path: Path) -> list[int]:
    text = path.read_text(encoding="utf-8")
    found = re.findall(
        r"6'd(\d+):\s*begin\s+bitmap_o\s*=\s*64'h([0-9A-Fa-f]+);",
        text,
    )
    templates = {int(index): int(bits, 16) for index, bits in found}
    if sorted(templates) != list(range(50)):
        raise RuntimeError(f"expected template addresses 0..49, got {sorted(templates)}")
    return [templates[index] for index in range(50)]


def classify(glyph: int, templates: list[int]) -> tuple[bool, int, int, int]:
    ranked = sorted(
        ((glyph ^ template).bit_count(), index)
        for index, template in enumerate(templates)
    )
    best_distance, best_index = ranked[0]
    second_distance = ranked[1][0]
    accepted = (best_distance <= MAX_DISTANCE and
                second_distance - best_distance >= MIN_MARGIN)
    return accepted, best_index, best_distance, second_distance


def main() -> int:
    templates = load_templates(ROM)
    failures: list[str] = []

    for expected, glyph in enumerate(templates):
        accepted, actual, distance, second = classify(glyph, templates)
        if not accepted or actual != expected or distance != 0:
            failures.append(
                f"exact class {expected + 1}: accepted={accepted} "
                f"actual={actual + 1} distance={distance} second={second}"
            )

    accepted_noisy = 0
    rejected_noisy = 0
    for expected, glyph in enumerate(templates):
        for bit in range(64):
            accepted, actual, distance, second = classify(
                glyph ^ (1 << bit), templates
            )
            if accepted and actual != expected:
                failures.append(
                    f"wrong noisy class {expected + 1}, bit {bit}: "
                    f"actual={actual + 1} distance={distance} second={second}"
                )
            elif accepted:
                accepted_noisy += 1
            else:
                rejected_noisy += 1

    if failures:
        print("SYMBOL TEMPLATE CHECK FAIL")
        print("\n".join(failures[:20]))
        return 1

    print(
        "SYMBOL TEMPLATE CHECK PASS: "
        f"50 exact, 3200 single-cell perturbations; "
        f"accepted={accepted_noisy}, safely_rejected={rejected_noisy}, wrong=0"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
