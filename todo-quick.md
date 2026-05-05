# NeuSLAM MP Benchmark Morning Checklist

Use this quick list to add node-side benchmarking in `neuslam.py` (no submodule edits).

- Import `time`, `deque`, and `TimeBenchmark` in `neuslam.py`.
- In `__init__`, create `self.bench = TimeBenchmark(enabled=True, use_cuda_events=False)`.
- In `__init__`, add `self._submit_wall_s = {}` and simple counters for callback/result rates.
- Before `self.slam.submit(batch)`, capture `frame_id = int(self.slam._submitted)` and store `time.perf_counter()` in `self._submit_wall_s[frame_id]`.
- After submit, keep `drain_results(non_blocking=True)` in callback (do not block).
- For each result message, map by `frame_id`, compute `dt_s = now - t_submit`, and call `self.bench.add("slam_step", dt_s)`.
- Keep publishing odometry as usual; do not change SLAM_MP internals.
- Add periodic logs (every 1-2s): callback Hz, result Hz, backlog (`_submitted - _received`), and avg/p95 latency.
- In shutdown path only (not callback): `end -> wait_until_done -> final drain -> save_outputs -> shutdown`.
- Write node benchmark report to file (e.g. `timing_node.txt` and optional `timing_node.json`).
- Validate: `slam_step` appears in report and FPS is non-null.

## Notes

- Do **not** call `end()/wait_until_done()/save_outputs()/shutdown()` per callback.
- Do **not** add CUDA sync for node-level FPS; wall-clock timing is the right metric.

