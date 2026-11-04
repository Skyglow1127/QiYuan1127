# DESC_ONLY BD: PS FCLK0 supplies the selected50/100MHz clock and BD/IP XDC
# carries its constraints. Do not invent create_clock on an internal net.
# No fabric ports are exported: descriptors, hard fault, CAN, GPIO and ILA
# probes are internal. Dedicated PS DDR/MIO use the verified board preset.
# TODO(pin): a future physical backend needs a reviewed carrier schematic,
# electrical interface, I/O voltage, reset/fault wiring and actual package pins.
# No PACKAGE_PIN, IOSTANDARD, false_path or DRC-severity override is authorized.
