# FPGA-Based Fitness Workout Timer

[cite_start]This repository contains the final project for the **Logic Circuits Laboratory (Spring 2025)** at **Amirkabir University of Technology (Tehran Polytechnic)**[cite: 509, 269]. The project is a comprehensive digital system for scheduling and managing personalized exercise routines, implemented in Verilog for an FPGA board.

## Project Overview

[cite_start]The system is designed to create and manage a workout session based on user-defined parameters[cite: 512]. [cite_start]It first calculates the required number of workout sets using a combinational logic unit and then controls the timing of exercise and rest periods through a Finite State Machine (FSM)[cite: 564, 326]. [cite_start]The entire design is implemented structurally without using behavioral multiplication or division operators, as per the project constraints[cite: 592].

### Core Features

#### 1. Workout Calculation Unit (Combinational)
[cite_start]This unit calculates the total number of one-minute workout sessions (T) based on the following formula[cite: 551, 277]:
$$ T = (\frac{Cal \times 60}{W}) \times G \times \frac{1}{MET} $$
User inputs are provided via switches:
* [cite_start]**Weight (W):** A 3-bit input representing weights from 50kg to 120kg[cite: 579, 6].
* [cite_start]**Target Calories (Cal):** A 2-bit input for targets of 50, 100, 150, or 200 kcal[cite: 575, 7].
* [cite_start]**MET (Metabolic Equivalent of Task):** A 2-bit input for intensity levels of 1, 2, 4, or 8[cite: 576, 7].
* [cite_start]**Gender (G):** A 1-bit input for male (`G=1`) or female (`G=1.125`) coefficients[cite: 577, 578, 7].

[cite_start]The implementation uses Look-Up Tables (LUTs), shifters, and adders to perform the calculation[cite: 283]. [cite_start]For example, the gender multiplier for females (1.125x) is implemented by adding the value to a right-shifted version of itself (`val + (val >> 3)`)[cite: 299, 70].

#### 2. Workout Scheduler (Finite State Machine)
[cite_start]A 5-state FSM manages the workout flow, controlling the transitions between exercising, resting, and completion states[cite: 3].
* [cite_start]Each workout session consists of **45 seconds of exercise** followed by **15 seconds of rest**[cite: 591].
* **States:**
    1.  [cite_start]`FSM_STATE_IDLE (000)`: Waits for the user to press the start button[cite: 9, 352].
    2.  [cite_start]`FSM_STATE_EXERCISING (001)`: 45-second countdown for the current exercise[cite: 10, 353].
    3.  [cite_start]`FSM_STATE_RESTING (010)`: 15-second countdown for the rest period[cite: 11, 354].
    4.  [cite_start]`FSM_STATE_BEEP_AFTER_REST (011)`: Generates a short beep to signal the end of rest and start of the next exercise[cite: 12, 355].
    5.  [cite_start]`FSM_STATE_FINAL_BEEP (100)`: Generates a longer, distinct beep to signal the completion of all workout sessions[cite: 12, 356].
* [cite_start]**Controls:** The FSM is controlled by three debounced buttons: `btn_start`, `btn_skip`, and `btn_reset`[cite: 608].

#### 3. I/O and Peripherals
* [cite_start]**7-Segment Display:** A 4-digit display shows the current exercise number and the remaining time for the work/rest period[cite: 53, 54]. [cite_start]During the idle state, it displays underscores[cite: 51, 52, 53].
* **Buzzer:** Provides audio feedback. [cite_start]A normal-frequency beep signals the end of a rest period, and a different frequency beep signals the final completion of the workout[cite: 4, 626].
* [cite_start]**LCD Display (Optional Feature):** A 16x2 LCD screen displays the name of the current exercise (e.g., "JUMP JACKS", "SIT UPS") and other real-time workout statistics[cite: 653, 56, 156, 157].

## File Structure

* `TopModule.v`: The main synthesizable Verilog module containing the top-level design and instantiations of all sub-modules.
* `TopModule_tb.v`: The Verilog testbench for simulating and verifying the design's functionality.
* `Logic_Lab_final_project_Spring2025-(Final)_2.pdf`: The original project specification document (in Persian).
* `Logic_Lab_Project.docx`: The detailed project report, including design methodology, FSM diagrams, and calculation logic (in Persian).

## Authors

* [cite_start]**Mohammadreza Hassanzadeh** [cite: 271]
* [cite_start]**Arian Ebrahimi** [cite: 271]

---
[cite_start]*This project was developed as a requirement for the Logic Circuits Laboratory course, supervised by Mr. Sagheb Haghighi.* [cite: 270]