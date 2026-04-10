# FPGA Implementation for Real-time Ultrasound Volterra Filtering

**Authors:** Divisht Dahiya, Helly Bera, Amartya Tuljapurkar   
Guidance: Prof. Joycee Mekie

---

## Project Overview
This repository contains a specialised signal-processing and hardware-engineering pipeline designed to enhance ultrasound image quality. The project focuses on Subharmonic B-mode imaging—a technique critical for high-contrast blood flow visualisation—and utilises Nonlinear Volterra Filtering to suppress noise and clutter.

The core of this project is the transition from high-level mathematical prototyping in MATLAB to low-latency execution on FPGA hardware, enabling real-time clinical application of complex nonlinear filters that are traditionally computationally expensive.
The pipeline focuses on using **Subharmonic Matched Filtering** and **Volterra Filtering** to isolate signals from ultrasound contrast agents. By moving these algorithms from **MATLAB** to an **FPGA**, we achieve the high-throughput processing required for real-time clinical diagnostics.

## 📁 Repository Structure
