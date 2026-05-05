# NeuSLAM MP Benchmark Plan (Node-Side Only)

Goal: measure end-to-end `SLAM_MP` FPS/latency from the ROS node without editing `submodules/Dynamic_SLAM`.

## 1) Add benchmark utilities to `neuslam.py`

- Import:
  - `time`
  - `from collections import deque`
  - `from core.utils.benchmark import TimeBenchmark`
- In `NeuSLAM.__init__`:
  - Create `self.bench = TimeBenchmark(enabled=True, use_cuda_events=False)`.
  - Add submit timestamp storage keyed by frame id:
    - `self._submit_wall_s = {}`
  - Add rolling counters/timers for operational metrics:
    - callback count + last print time
    - result/odom count + last print time
    - optional deque for recent latencies

## 2) Track submit time per frame

- In `slam_callback`, right before `self.slam.submit(batch)`:
  - Read current frame id from `self.slam._submitted` (private but practical):
    - `frame_id = int(self.slam._submitted)`
  - Store submit wall time:
    - `self._submit_wall_s[frame_id] = time.perf_counter()`
  - Then call `self.slam.submit(batch)`.

## 3) Compute end-to-end step latency and FPS

- Where you process `for msg in self.slam.drain_results(non_blocking=True):`
  - For each `msg["cmd"] == "result"`:
    - Get `frame_id = int(msg.get("frame_id", -1))`
    - Pop submit time from dict:
      - `t0 = self._submit_wall_s.pop(frame_id, None)`
    - If `t0` exists:
      - `dt_s = time.perf_counter() - t0`
      - `self.bench.add("slam_step", dt_s)`
      - optional: add to rolling latency deque
  - Keep existing `_publish_pose(msg)`.

Why this works:

- `TimeBenchmark` computes FPS from the `slam_step` series.
- This gives true node-observed end-to-end pipeline timing (submit -> result).

## 4) Add periodic runtime logs (optional but useful)

Every ~1-2 seconds print:

- callback/input rate (Hz)
- result/odom publish rate (Hz)
- backlog:
  - `int(self.slam._submitted) - int(self.slam._received)`
- latency summary from recent samples (mean/p95)

This helps diagnose queue buildup and async behavior.

## 5) Save/report benchmark on shutdown

In node shutdown/finalization path (for MP mode):

- Call in order:
  - `self.slam.end()`
  - `self.slam.wait_until_done()`
  - drain remaining results and record `slam_step` timings
  - `self.slam.save_outputs(<output_dir>)`
  - `self.slam.shutdown()`
- Write node benchmark report to file, e.g.:
  - `<output_dir>/timing_node.txt` with `self.bench.format_report()`
- Optional JSON:
  - `self.bench.save_json(<output_dir>/timing_node.json)`

Note:

- Keep this shutdown sequence out of the per-frame callback.

## 6) What NOT to do

- Do not call `end()/wait_until_done()/save_outputs()/shutdown()` inside each callback.
- Do not add CUDA synchronize/timer calls for node-level FPS.
  - Wall-clock timing is the right metric for end-to-end throughput.

## 7) Quick validation checklist

- Node runs with MP enabled and no callback blocking regressions.
- `slam_step` appears in node benchmark report.
- Reported FPS is non-null and reasonable.
- Backlog stabilizes (or clearly indicates overload).
- Output files are written on clean shutdown.

