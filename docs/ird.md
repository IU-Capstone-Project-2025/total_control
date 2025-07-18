# 📄 Idea Proposal Document

## 🛠️ **Project Title:**  
**Inno Control** — Python Library for learning control theory with real hardware.

---

## 📌 **Introduction**

- **Project Title:** Inno Control  
- **Author(s):** Artyom Tuzov *(Dungeon team)*  
- **Date:** 01.06.2025  
- **Company/Department:** Innopolis University  

This document describes the **Inno Control** project — a Python library and ready-to-use methodology for a fast start in practical control theory education.
It is designed for students in the Robotics track at Innopolis University and provides a simple, intuitive way to connect to physical setups (e.g., the *Cart-Pole* system) without complicated low-level configuration.

---

## 🗂️ **Executive Summary**

**Inno Control** allows students to work with real physical control systems without wasting time configuring controllers or writing embedded firmware in C/C++. The connection works in **plug & play** mode using:  

- A Python library  
- Detailed API documentation (*Sphinx*)  
- Easy-to-use data exchange methods  

---

## ❗ **Problem Statement**

**Problem:**  
- Robotics students spend a lot of time configuring firmware, debugging controllers, and manually setting up hardware.  
- The entry barrier for hands-on control experiments is high due to the need to know C/C++ for embedded development.  
- There is no universal, user-friendly Python library that *just works* with setups like Cart-Pole out of the box.  
- Robotics students can’t apply their knowledge in practice, on real robots and hardware.

**Evidence:**  
- Students face many difficulties when launching lab hardware for the first time.  
- Existing solutions are too specialized and lack a unified **plug & play** format for control experiments.

---

## 💡 **Proposed Solution**

**Inno Control Features:**  
- ✅ Flexible Python library for connecting to physical control systems (e.g., *Cart-Pole*) through a simple, intuitive API.  
- ✅ Detailed **Sphinx-based** documentation with clear examples so any student can launch a setup in less than a minute.  
- ✅ Clean command interface: easy functions for device initialization, starting experiments, reading data, and setting control efforts.  
- ✅ Plug & play **USB/UART** connection with minimal configuration.

---

## ⚙️ **Technical Feasibility**

**Tech Stack:**  
- **Language:** Python 3.x  
- **Documentation:** Sphinx + reStructuredText  
- **Firmware:** Minimal ESP32 firmware that processes commands and streams sensor data.  
- **Protocol:** Serial UART

---

## 🎓 **Benefits and Value**

- **Educational impact:** Students dive into real control tasks immediately — without barriers related to firmware development.  
- **Motivation boost:** Less time spent debugging — more time for experiments, creativity, and real understanding of control principles.  
- **Scalability:** Instructors and students can easily add new modules for other hardware setups.

---

## 🗓️ **Implementation Plan**

**Timeline:** 6 weeks total

| Week | Task                       |
|------|----------------------------|
| 1-2  | Firmware & hardware debugging |
| 3-4  | MVP development            |
| 5    | Write documentation        |
| 6    | Testing                    |

**Budget:**  
- **Development:** $0  
- **Hardware:** Already available

---

## ✅ **Conclusion**

**Inno Control** addresses a critical educational need by removing technical barriers to learning practical control theory. The project aligns with **Innopolis University’s** mission to promote hands-on learning.
