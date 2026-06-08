# PostureSense 🧘‍♂️

A high-precision IoT posture monitoring system using an ESP32-S3 and three IMUs to track spinal curvature in real-time.

## 🚀 Project Overview
PostureSense uses three inertial measurement units (IMUs) mounted along the spine (Upper, Mid, Lower) to detect clinical posture deviations. It processes data locally at 4Hz and streams telemetry to a web-based dashboard.

---

## 🧠 How it Works: The "Digital Spine"

The system treats the spine as a segmented chain of three links. By comparing the **relative angles (Deltas)** between these segments, it can identify specific clinical conditions regardless of whether the user is sitting or standing.

### 1. The Calibration (The "Zero Point")
During calibration, the device records the "perfect" posture of the user. All subsequent readings are compared against this baseline:
`Current Angle - Baseline Angle = Deviation`

### 2. Detection Algorithms
| Condition | Logic | Physical Meaning |
| :--- | :--- | :--- |
| **Thoracic Slouch** | `(Upper - Mid) Pitch > 10°` | Excessive forward rounding of the upper back (C-curve). |
| **Forward Flexion** | `Upper Pitch > 20°` | Proxy for "Text Neck"; head and neck leaning forward. |
| **Lateral Lean** | `Roll Spread > 10°` | Side-to-side spinal deviation or leaning. |
| **Lumbar Hyperlordosis**| `(Mid - Lower) Pitch > 10°` | Excessive inward arching of the lower back. |
| **Lumbar Flattening** | `(Mid - Lower) Pitch < -10°` | Slumping; loss of natural lumbar curve (posterior pelvic tilt). |

### 3. The EMA Filter (Smoothing)
To prevent "alert fatigue" from natural movement (like breathing or reaching), the system uses an **Exponential Moving Average (EMA)**:
- **Score (0.0 to 1.0):** Each condition has a probability score.
- **Persistence:** You must maintain a bad posture for **~5 seconds** for the score to cross the **0.65 alert threshold**.
- **Forgiveness:** Short movements do not trigger alerts.

---

## 📂 Project Structure

- **`posture_sense.ino`**: The main firmware for the Heltec ESP32-S3. Handles I2C multiplexing, sensor fusion (Madgwick), and detection logic.
- **`index.html`**: The frontend dashboard. Visualizes EMA scores, provides calibration controls, and alerts the user.
- **`mock_server.py`**: A Python-based WebSocket server that simulates the hardware for testing without the wearable.
- **`IoTProposalPresentation.pdf`**: Technical specifications and research background.

---

## 🛠 Setup & Usage

### 1. Using the Mock Server (Simulation)
1. Install dependencies: `pip install websockets`.
2. Run the server: `python mock_server.py`.
3. Open `index.html`. It will automatically connect to `ws://localhost:8765`.

### 2. Using the Real Hardware
1. Flash `posture_sense.ino` to your ESP32-S3.
2. **Wireless Mode:** Connect to the WiFi AP `PostureSense` (Password: `12345678`).
3. Open `http://posture.local` (or `192.168.4.1`) in your browser.

---

## 🛠 Technical Stack
- **Hardware:** ESP32-S3 (Heltec), TCA9548A Multiplexer, 3x ICM-20948 IMUs.
- **Firmware:** C++ (Arduino), MadgwickAHRS, ArduinoJson, ESPAsyncWebServer.
- **Frontend:** Vanilla JS, CSS3, HTML5 (WebSockets).
- **Mocking:** Python (Asyncio).
