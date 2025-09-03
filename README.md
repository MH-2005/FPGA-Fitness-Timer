# FPGA-Based Fitness Workout Timer

This repository contains the final project for the **Logic Circuits Laboratory (Spring 2025)** at **Amirkabir University of Technology (Tehran Polytechnic)**. The project is a comprehensive digital system for scheduling and managing personalized exercise routines, implemented in Verilog for an FPGA board.

## 🏋️ Project Overview

The system is designed to create and manage a workout session based on user-defined parameters. It first calculates the required number of workout sets using a combinational logic unit and then controls the timing of exercise and rest periods through a Finite State Machine (FSM). The entire design is implemented structurally without using behavioral multiplication or division operators, as per the project constraints.

The project is divided into two main parts:
1.  **Calculation Unit:** A purely combinational circuit that takes user inputs (weight, gender, target calories, and exercise intensity) and calculates the total number of one-minute workout sessions required.
2.  **Scheduler Unit:** A sequential circuit, implemented as a 5-state Finite State Machine (FSM), that manages the timing for each workout and rest period, controls the user interface (display and buzzer), and handles user interaction through buttons.

---

## ✨ Core Features

### 1. Workout Calculation Unit (Combinational)
This unit calculates the total number of one-minute workout sessions (T) based on the following formula:
$$ T = (\frac{Cal \times 60}{W}) \times G \times \frac{1}{MET} $$
User inputs are provided via slide switches on the FPGA board:
* **Weight (W):** A 3-bit input representing weights from 50kg to 120kg.
* **Target Calories (Cal):** A 2-bit input for targets of 50, 100, 150, or 200 kcal.
* **MET (Metabolic Equivalent of Task):** A 2-bit input for intensity levels of 1, 2, 4, or 8.
* **Gender (G):** A 1-bit input for male (`G=1`) or female (`G=1.125`) coefficients.

To adhere to the project's constraints, the calculation is performed without using `*` or `/` operators. Instead, it relies on:
* **Look-Up Tables (LUTs):** The `(Cal * 60) / W` portion is pre-calculated for all 32 possible combinations of `Cal` and `W` and stored in a combinational block that acts as a ROM.
* **Shifters and Adders:**
    * The gender multiplier for females (1.125x) is efficiently implemented by adding the value to a version of itself shifted right by 3 bits (`val + (val >> 3)`), which is equivalent to `val + val/8`.
    * The division by MET is implemented using right-shifts, as all MET values are powers of two (`>> 0` for 1, `>> 1` for 2, `>> 2` for 4, `>> 3` for 8).

### 2. Workout Scheduler (Finite State Machine)
A 5-state FSM manages the workout flow, controlling the transitions between exercising, resting, and completion states.
* Each workout session consists of **45 seconds of exercise** followed by **15 seconds of rest**.
* **States:**
    1.  `FSM_STATE_IDLE (000)`: Waits for the user to press the start button. In this state, the 7-segment display shows underscores, and the LCD displays the user-selected parameters and the calculated total number of exercises.
    2.  `FSM_STATE_EXERCISING (001)`: A 45-second countdown timer runs for the current exercise.
    3.  `FSM_STATE_RESTING (010)`: A 15-second countdown timer runs for the rest period.
    4.  `FSM_STATE_BEEP_AFTER_REST (011)`: A transient state that generates a short beep to signal the end of the rest period and the start of the next exercise.
    5.  `FSM_STATE_FINAL_BEEP (100)`: A state entered upon completion of all workout sessions, which generates a longer, distinct beep to signal the end of the entire workout.
* **Controls:** The FSM is controlled by three debounced push-buttons: `btn_start`, `btn_skip` (to skip the current exercise or rest period), and `btn_reset`.

### 3. I/O and Peripherals
* **7-Segment Display:** A 4-digit display shows the current exercise number and the remaining time for the work/rest period. During the idle state, it displays underscores (`____`).
* **Buzzer:** Provides audio feedback. A normal-frequency beep signals the end of a rest period, and a different, lower-frequency beep signals the final completion of the workout.
* **LCD Display (Optional Feature):** A 16x2 LCD screen displays the name of the current exercise (e.g., "JUMP JACKS", "SIT UPS") and other real-time workout statistics, such as the current exercise number out of the total (`E01/28`) and the remaining time.

---

## 📂 File Structure

```

.
├── TopModule.v                  \# Main synthesizable Verilog module with all sub-modules instantiated.
├── ExerciseCalculator\_tb.v      \# Verilog testbench for simulating and verifying the calculation unit.
├── inputs.txt                   \# Input vectors for the simulation testbench.
├── output.txt                   \# Simulation results, comparing RTL output against a reference model.
├── pins.txt                     \# Pin constraints file for the target FPGA board.
├── Logic\_Lab\_final\_project\_Spring2025-(Final).pdf  \# Original project specification document (Persian).
├── G05\_40226046\_40231001.pdf     \# Detailed project report with design methodology and diagrams (Persian).
└── README.md                    \# This file.

```

---

## 🛠️ Getting Started

### Prerequisites
* A Verilog simulator (e.g., Icarus Verilog, ModelSim, or the simulator included in Xilinx ISE/Vivado).
* (For hardware implementation) A supported FPGA development board and the corresponding software (e.g., Xilinx ISE or Vivado).

### Simulation
The functionality of the `ExerciseCalculator` module can be verified using the provided testbench.
1.  Ensure `inputs.txt` is in the same directory as the Verilog source files. This file contains the test vectors.
2.  Compile `ExerciseCalculator.v` (part of `TopModule.v`) and `ExerciseCalculator_tb.v` with your simulator.
3.  Run the simulation.
4.  The testbench will read each line from `inputs.txt`, apply the inputs to the DUT, calculate a reference value, and write the comparison to `output.txt`. It will also print a summary of passed and failed tests to the console.

### Hardware Implementation
1.  Create a new project in your FPGA development software (e.g., Xilinx ISE).
2.  Add `TopModule.v` as the top-level design file.
3.  Add the `pins.txt` file as the user constraints file to map the inputs and outputs to the correct physical pins on the FPGA.
4.  Run the synthesis and implementation process.
5.  Upload the generated bitstream file to the FPGA board.
6.  Use the slide switches to set your workout parameters (Weight, Calories, MET, Gender) and the push buttons to control the workout session.

---

## 👥 Authors

* **Mohammadreza Hassanzadeh**
* **Arian Ebrahimi**

This project was developed as a requirement for the Logic Circuits Laboratory course, supervised by **Mr. Sagheb Haghighi**.

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
