# 🌿 AGROSENSE powered by AWLAK NANE -- Smart IoT-Based Farm Monitoring System

This project connects a **remote farm** to a **central home dashboard** using **LoRa** and **Wi-Fi** technologies, enabling farmers to monitor and control environmental conditions such as temperature, humidity, and soil moisture. It features real-time data visualization, automatic irrigation control, and a custom-built Node-RED dashboard.

---

## 📁 Project Structure

### 🏠 Home (Main Farm) Environment

- **File:** `main.cpp`
- **Platform:** ESP32
- **Description:**
  - Receives data from the remote farm via LoRa.
  - Sends data to the Node-RED dashboard via Wi-Fi.
  - Displays live sensor readings on the OLED display.

### 🚜 Remote Farm Environment

- **File:** `arduino.cpp` (located in `arduino_code/`)
- **Platform:** Arduino Mega
- **Description:**
  - Reads temperature, humidity, and soil moisture sensors.
  - Sends data to the ESP32 (main farm) using LoRa.
  - Controls the water valve based on automatic/manual commands.

---

## 🌐 Dashboard Setup (Node-RED)

- **File:** `flows.json` (located in `node_red/`)
- **Instructions:** Import this file into your local Node-RED editor to launch the visual dashboard.

### 📊 Dashboard Tabs Overview

| Tab Name              | Description                                                                 |
|-----------------------|-----------------------------------------------------------------------------|
| **Home**              | Displays real-time data from the main farm.                                |
| **Remote Farm**       | Displays real-time data from the remote farm.                              |
| **Control**           | Allows threshold-based auto irrigation control for the remote farm.         |
| **History**           | Graphs historical temperature, humidity, and soil moisture (main farm).     |
| **Remote Farm History** | Graphs historical data from the remote farm.                              |
| **About**             | Learn more about this IoT project and the amazing team behind it.           |

---

## ⚙️ How to Run

### 1. Set up the Remote Farm

- Upload `arduino_code/arduino.cpp` to your **Arduino Mega**.
- Power on the sensor suite and valve system.

### 2. Set up the Home (Main Farm)

- Upload `main.cpp` to your **ESP32**.
- Connect to Wi-Fi and ensure the OLED display and LoRa modules are connected.

### 3. Launch the Dashboard

- Open **Node-RED** on your system.
- Import `flows.json`.
- Navigate to `http://localhost:1880/ui` to access the dashboard.

---

## 💡 Features

- Long-range wireless communication using LoRa.
- Real-time sensor data visualization.
- Automatic and manual irrigation control.
- Historical data charting.
- OLED display for local feedback.
- Modular and scalable design.

---

## 👩‍💻 Team

> _Meet the brilliant minds behind this project in the **About** tab on the dashboard._

---

## 📜 License

This project is licensed under the MIT License. Feel free to fork, use, and contribute!

