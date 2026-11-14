# 1D altitude Kalman filter in MATLAB

A hands-on, guided state-estimation learning project by Dhruv Mallick. A two-state linear Kalman filter estimates height and vertical velocity using synthetic barometer readings and an accelerometer input.

This is an offline simulation, not a flight-tested estimator, PX4 integration, or hardware experiment. The trajectory and sensor noise are generated in base MATLAB.

## Original code

`step1_truth.m` contains the completed script from the final experiment. The web editor removed its leading blank line; no MATLAB code was changed. A byte-for-byte original is retained in the accompanying portfolio archive. Despite its early-stage filename, it includes the trajectory, noisy sensors, accelerometer-only integration, Kalman prediction/correction, dropout handling, plots, and RMSE calculations.

The checked-in state is intentionally the last experiment: an incorrect 5 m initial height with very small initial covariance. It is not the best-performing baseline configuration.

## Model

- Duration: 40 s; sample interval: 0.1 s.
- Synthetic trajectory: climb at 0.25 m/s, hover, then descend at -0.25 m/s.
- State: height and vertical velocity.
- Transition: `F = [1 dt; 0 1]`.
- Acceleration input: `B = [0; dt]`.
- Barometer observation: `H = [1 0]`.
- Noise standard deviations: 0.3 m for height and 0.5 m/s^2 for acceleration.
- Default process covariance: `Q = diag([0.01 0.01])`; barometer variance: `R = 0.09`.
- Seed: `rng(1)`.

## Run

Open `step1_truth.m` in MATLAB or MATLAB Online and run the whole script. No additional toolbox is used. Inspect `rmse_baro`, `rmse_acc`, and `rmse_kf` in the workspace, or enter:

```matlab
[rmse_baro, rmse_acc, rmse_kf]
```

For clean plots between whole-script runs, enter `close all` first. The original contains repeated plotting calls with `hold on`; this is preserved, not silently rewritten.

## Experiments and observed results

Results below were read from the author's MATLAB runs on November 14, 2026. They are observations, not a claim of a fresh independent MATLAB rerun of this repository.

| Configuration | Height RMSE, m |
| --- | ---: |
| Barometer only, full readings | 0.2866 |
| Accelerometer double integration | 2.8179 |
| Baseline Kalman filter | 0.1394 |
| Kalman filter with missing barometer readings | 0.1546 |
| Wrong 5 m start, initial P = eye(2) | 0.1415 |
| Wrong 5 m start, initial P = 1e-4*eye(2) | 0.3787 |

To reproduce each comparison, keep `rng(1)` and the default Q/R active. Make a copy of the original and change only the indicated configuration:

1. **Baseline:** `x = [0; 0]; P = eye(2);` and keep the dropout assignment commented.
2. **Dropout:** baseline initial state/covariance, then uncomment `h_baro(201:250) = NaN;`. These 50 samples cover 20.0 through 24.9 s. The filter skips correction when the barometer is missing, but continues prediction. The original barometer RMSE becomes NaN in this case because its formula includes missing values. For a separate available-readings score, use `valid = ~isnan(h_baro); sqrt(mean((h_baro(valid)-h(valid)).^2))`; do not confuse that partial-data metric with full-trajectory filter RMSE.
3. **Wrong start, ordinary uncertainty:** `x = [5; 0]; P = eye(2);`, dropout commented.
4. **Wrong start, high confidence:** `x = [5; 0]; P = 0.0001*eye(2);`, dropout commented. This is the checked-in original.
5. **Q/R tuning:** the two alternative commented Q/R pairs in the script were used to examine excessive model trust versus excessive sensor trust. Change one pair at a time; keep the baseline pair commented while an alternative is active. No numerical results are claimed here for those tuning variations.

## Plot evidence

Four cropped MATLAB screenshots accompany the portfolio archive: baseline, barometer dropout, wrong initial height with ordinary covariance, and wrong initial height with very small covariance. Each includes the observed plot and workspace RMSE values, with desktop/account details removed. They are not regenerated simulation pictures. The repository's numerical table records those observations.

## What the experiments show

The baseline filter reduces noise without the growing drift of accelerometer-only integration. Missing measurements leave the estimator predicting until corrections resume. Initial covariance expresses assumed uncertainty, not actual error: a small P can make a wrong starting state slower to correct. Q and R control the balance between model and measurement uncertainty.

## Limits and next steps

The simple synthetic model omits sensor bias, attitude, gravity compensation, real vehicle dynamics, and real flight logs. A favorable RMSE on this one seeded trajectory does not establish performance on a drone. The current script stores the height estimate but does not plot or score its velocity estimate. Broader testing and a physically richer sensor model would be separate future work.

## Learning context

Developed as a guided MATLAB exercise with explanatory and debugging assistance. The completed script and measured experiments document practical learning, rather than a claim that this estimator or its mathematics was invented independently.
