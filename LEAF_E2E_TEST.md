# Leaf “working” end-to-end test (minimal)

Goal: repeatable checklist to prove **data preservation** on a leaf node.

## Preconditions
- Leaf firmware built and ready to flash.
- Serial terminal connected (115200 or project default).
- SD card available (known-good, formatted).
- Root node available for mesh validation.

## Test checklist

### 1) Flash leaf in RUN mode
- Flash the leaf and reset into **RUN** mode.
- Confirm boot completes without errors.

### 2) Verify continuous serial JSON
- Observe serial output for **continuous JSON lines**.
- Confirm records increment and do not pause under normal conditions.

### 3) Remove SD card (or boot without it)
- With leaf running, **remove the SD card** (or power-cycle with SD removed).
- **Expected:** Serial JSON continues uninterrupted.
- **Expected:** Device buffers data to FRAM (no loss of serial output).

### 4) Insert SD and reboot
- Insert SD card and **reboot** the leaf.
- **Expected:** Buffered records flush to SD (best-effort).
- **Expected:** Serial output remains continuous.

### 5) Bring up root
- Power the root node and bring mesh online.
- **Expected:** Leaf joins the mesh.
- **Expected:** Root prints leaf records.

### 6) Set TZ/DST and verify localtime
- Configure TZ/DST on the leaf (or via root if that is the flow).
- **Reboot** the leaf.
- **Expected:** File timestamps and localtime behavior match configured TZ/DST.

## Acceptance criteria
- **Leaf never stops emitting serial records** in any failure mode tested.
- **Data preservation is demonstrated** via FRAM buffering and SD flush on recovery.
- **Mesh visibility confirmed** by root printing leaf records.
- **Localtime correctness confirmed** after TZ/DST configuration and reboot.
