# Figure contract

The figures test whether the published window-size effects can be recovered under explicit simulation assumptions. They are engineering validation plots, not submission-ready claims of numerical reproduction.

Backend: Python, following the established project workflow. Export: 180 mm width, 105 mm height, 8 pt labels, editable PDF/SVG, PNG preview. Each figure has one axes; panel alignment is not applicable but still recorded by the audit helper. Use restrained blue/orange plus neutral markers, with line style and shape redundancy.

- Dynamic figure: all seven published M values, the paper's printed peak-error labels, computed errors evaluated at the window mean time and at the newest event. The model is noiseless apart from 200 MHz quantization; the unspecified background/phase and error-sampling definition preclude exact equivalence. The M=1024 disagreement must remain visible. No independent-replicate uncertainty applies to this deterministic run.
- Static figure: 32 independent noisy acquisitions, fixed amplitude/SNR convention, 10 kHz noise bandwidth and ±0.05 A0 comparator thresholds (declared assumptions). Plot the RMS of the per-frame across-record standard deviation and the total RMSE. Curves summarize all valid frames; overlapping frames are not independent replicates. A bootstrap over whole independent records provides a 95% interval for both metrics. Missing/extra edges are reported, not silently filtered.

Do not digitize or redraw the paper's full figures as if they were new measurements. The seven manually transcribed labels are identified as paper values. No original PDF pages are included in the output package.
# Supplementary simulation checks (2026-11-11)

Four original Python figures, PDF/SVG plus 300 dpi PNG previews, 180 mm width.
They report simulated data, not author measurements or certified reproduction.

- Theory: independent timing-noise Monte Carlo supports the variance formula and fair equal-window SFLS/EP comparison. 2048 independent trials; both methods use 99.9% chi-square STD intervals. Single axis.
- Frontend sensitivity: STD and RMSE show different dependencies on noise and comparator threshold. Two equal panels, 16 independent records and 95% whole-record bootstrap intervals. All eight fixed-10-kHz cases are shown; two auxiliary bandwidth cases remain in the source results and report because bandwidth is held constant in this figure.
- Drift map: different reference times change the apparent tracking error. Two aligned panels with identical color scale; each cell is one evaluated frame, no fitted color interpolation; RMSE across 16 independent records. Colorbar is excluded from panel comparison because it encodes the shared scale rather than another experiment. This is a declared-assumption reconstruction, not extracted Fig.8 data.
- Step trace: ideal 200 MHz quantization permits small-step tracking in a conditional model. Raw overlapping outputs are shown; no independent-sample CI is inferred from them. Noise sensitivity is evaluated separately using 512 independent acquisitions per level. Single axis.

Every rendered PDF must pass the 5 pt glyph floor and collision audit; comparable plot rectangles pass the 1.5 pt alignment gate. Raw observations remain in JSON. These are research-review figures, not a journal submission package.

