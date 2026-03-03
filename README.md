# Portal Book Nook Arduino Sketches

Arduino sketches for the Portal Book Nook project, including ESP32-CAM stream sender and CYD display receivers.

## Current focus
- `PortalOrangeCam/PortalOrangeCam.ino`: ESP32-CAM SoftAP (`BluePortal`) with:
  - TCP JPEG stream for CYD on port `5000`
  - Web tuning UI on port `80`
  - Persistent camera settings (NVS / Preferences)

## Notes
- Camera pin mapping is for AI-Thinker ESP32-CAM.
- The project is under active tuning and diagnostics.
