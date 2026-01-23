Using SciPy is a solid choice. And yes: **use second-order sections (SOS) + forward–backward filtering** (`sosfiltfilt`) for an offline dataset—SciPy explicitly recommends SOS for numerical stability, and `sosfiltfilt` is the SOS version of zero-phase filtering. ([docs.scipy.org][1])

### Do you “only need to filter acceleration”?

Not really. If you **compute** acceleration from measured signals, the “damage” happens **during differentiation**: numerical differentiation amplifies noise and is ill-posed in the presence of measurement noise. ([ResearchGate][2])
Filtering *only* the resulting (\ddot q) helps cosmetically, but it often leaves you with:

* noisy derivatives leaking into training targets,
* inconsistent phase/amplitude vs. torque,
* and higher variance across runs.

### What I would do (simple + standard, still Butterworth)

Given you have **position (q)**, **velocity (\dot q)**, and **effort/torque (\tau)** from the robot:

#### Best “low effort / high payoff” pipeline

1. **Filter (q) and (\dot q)** with the same 4th-order Butterworth low-pass (SOS + `sosfiltfilt`).
2. Compute (\ddot q) by differentiating the **filtered (\dot q)** (central differences).
3. Optionally **filter (\ddot q)** once more with the same filter (often improves stability).
4. Also filter **(\tau)** with the same low-pass (same cutoff) so your inputs/targets share bandwidth.

This keeps all signals in a comparable frequency band (important for inverse dynamics), while preventing differentiation from turning encoder noise into “acceleration dynamics”.

#### When is “filter only acceleration” acceptable?

* If the robot already provides a **physically estimated/filtered acceleration** (not just finite differences), then yes, you can often just use it.
* But if your acceleration is derived (which it was previously), it’s better to **filter before differentiating**, not only after.

### SciPy specifics you’re already aligned with

* Design filter with `butter(..., output="sos")` → recommended by SciPy to avoid numerical issues with higher-order IIR in direct form. ([docs.scipy.org][3])
* Apply with `sosfiltfilt(sos, x)` for **zero-phase** offline filtering. ([docs.scipy.org][4])

### The only real tuning knob: cutoff frequency

Order “4” is fine; the cutoff matters most. Pick it based on sampling rate and your motion bandwidth (often start around **5–30 Hz** for robot joint trajectories, then validate by spectra/plots).

If you tell me your **sampling rate (Hz)** and roughly how “fast” the trajectories are (slow/medium/aggressive), I’ll suggest a concrete cutoff (and a quick sanity checklist: plots to confirm you’re not over-smoothing or under-filtering).

[1]: https://docs.scipy.org/doc/scipy-1.16.2/reference/generated/scipy.signal.filtfilt.html?utm_source=chatgpt.com "filtfilt — SciPy v1.16.2 Manual"
[3]: https://docs.scipy.org/doc/scipy-1.16.2/reference/generated/scipy.signal.butter.html?utm_source=chatgpt.com "butter — SciPy v1.16.2 Manual"
[4]: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.sosfiltfilt.html?utm_source=chatgpt.com "sosfiltfilt — SciPy v1.17.0 Manual"