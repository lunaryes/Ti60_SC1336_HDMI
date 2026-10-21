"""Compile and run camera OCR fixtures without non-ASCII readmemh paths."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work-dir", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--generated-dir", type=Path, help="Template and fixture output; defaults to generated/ in staging or sim/build_ocr/ in the repository.")
    parser.add_argument("--iverilog-dir", type=Path, default=Path("F:/codex/tools/iverilog/bin"))
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--result-name", default="ocr_camera_results", help="Output JSON stem; preserve separate test snapshots.")
    parser.add_argument("--name", action="append", help="Run only fixtures containing this substring.")
    args = parser.parse_args()
    root = args.work_dir.resolve()
    generated = (args.generated_dir or (root / "generated" if (root / "generated").is_dir() else root / "sim" / "build_ocr")).resolve()
    output_dir = generated / "tests"
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / (args.result_name + ".vvp")
    source_dir = root / "src"
    if not (source_dir / "ocr_sampler16.v").is_file():
        source_dir = root / "Ti60F225_SC1336_HDMI_Display" / "src" / "recognition"
    sources = [source_dir / name for name in ("ocr_sampler16.v", "ocr_matcher16.v", "simple_shape_ocr.v")]
    sources += [generated / "ocr_font16_rom.v", root / "sim" / "tb_ocr_camera.v"]
    source_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    compiler = args.iverilog_dir / "iverilog.exe"
    runtime = args.iverilog_dir / "vvp.exe"
    command = [str(compiler), "-g2012", "-s", "tb_ocr_camera", "-o", str(output), *map(str, sources)]
    compiled = subprocess.run(command, capture_output=True, text=True)
    print(compiled.stdout + compiled.stderr, end="", flush=True)
    if compiled.returncode:
        return compiled.returncode
    print("COMPILE g2012 PASS", flush=True)
    compatibility = subprocess.run([str(compiler), "-g2005", "-s", "simple_shape_ocr", "-t", "null", *map(str, sources[:-1])], capture_output=True, text=True)
    print("COMPILE g2005 " + ("PASS" if compatibility.returncode == 0 else "FAIL"), flush=True)
    print(compatibility.stdout + compatibility.stderr, end="", flush=True)
    fixtures = json.loads((generated / "ocr_stimuli_manifest.json").read_text(encoding="utf-8"))["fixtures"]
    if args.name:
        fixtures = [fixture for fixture in fixtures if any(name in fixture["name"] for name in args.name)]

    def run_one(fixture: dict) -> dict:
        background = 0 if fixture["polarity"] == "light_on_dark" else 255
        command = [str(runtime), str(output), "+STIM=" + fixture["hex_file"], "+EXPECT=" + str(fixture["expected_ascii"]), "+BG=" + str(background)]
        started = time.monotonic()
        result = subprocess.run(command, cwd=generated, capture_output=True, text=True, timeout=300)
        failure = re.search(r"OCR FAIL expected=(.) actual=(.) recognized=([01])", result.stdout)
        if result.returncode == 0:
            outcome = "correct" if fixture["expected_ascii"] else "negative_rejected"
        elif failure is not None:
            outcome = "rejected" if failure.group(3) == "0" else "wrong_class"
        elif "negative scene accepted as" in result.stdout:
            outcome = "negative_accepted"
        else:
            outcome = "test_error"
        return {"name": fixture["name"], "passed": result.returncode == 0, "returncode": result.returncode,
                "outcome": outcome, "expected_ascii": fixture["expected_ascii"],
                "allow_reject": fixture.get("allow_reject", False),
                "input_bbox_exclusive": fixture.get("foreground_bbox_exclusive"),
                "seconds": round(time.monotonic() - started, 2), "stdout": result.stdout, "stderr": result.stderr}

    records = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        tasks = {pool.submit(run_one, fixture): fixture["name"] for fixture in fixtures}
        for task in as_completed(tasks):
            try:
                result = task.result()
            except Exception as exception:
                result = {"name": tasks[task], "passed": False, "error": str(exception)}
            records.append(result)
            print(("PASS " if result["passed"] else "FAIL ") + result["name"], flush=True)
            print(result.get("stdout", "") + result.get("stderr", "") + result.get("error", ""), end="", flush=True)
    outcomes = {name: sum(record.get("outcome") == name for record in records)
                for name in ("correct", "rejected", "wrong_class", "negative_rejected", "negative_accepted", "test_error")}
    (output_dir / (args.result_name + ".json")).write_text(json.dumps({
        "verilog2005_passed": compatibility.returncode == 0, "source_sha256": source_hashes,
        "outcomes": outcomes, "fixtures": records}, indent=2) + "\n", encoding="utf-8")
    passed = sum(record["passed"] for record in records)
    print(f"TOTAL {passed}/{len(records)} PASS; g2005={'PASS' if compatibility.returncode == 0 else 'FAIL'}", flush=True)
    print("ACTUAL OUTCOMES " + json.dumps(outcomes), flush=True)
    print("Ambiguous-character rejection remains a strict test failure; no expected label is changed.", flush=True)
    return 0 if passed == len(records) and compatibility.returncode == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
