# OTTO (Overdrive Tactical Transforming Operator Robot)

**This repository is currently under active development.** Many sections, configurations, and images are subject to change as progress is made. Expect frequent updates, including:

- New mechanical designs and optimized subassemblies.
- Improved CANBus configurations and motor tuning.
- Updated URDF files for better simulation accuracy.
- Additional images and diagrams to illustrate the robot’s structure and posture.



If you have suggestions or find issues, feel free to contribute or report them in the repository's issue tracker.

![otto_sleeping.jpg](/images/CAD/OTTO_Sleeping.jpg)

## Table of Contents

- [Components](#componenets)
- [Mechanical Assembly](#mechanical-assembly)
   - [SolidWorks Modeling](#solidworks-modeling)
   - [Exporting STL for URDF](#exporting-stl-for-urdf)
- [CyberGear Motor CANBus Configuration](#cybergear-motor-canbus-configuration)
- [Basic Posture of the Robot](#basic-posture-of-the-robot)
- [Material Specifications](#material-specifications)

---

## Componenets

[2-CH CAN HAT+](https://www.waveshare.com/wiki/2-CH_CAN_HAT+)

2-CH CAN HAT+ is an isolated expansion board for Raspberry Pi, supports dual-channel CAN communication, and features multi-protection circuits, wide voltage input, and so on.
![2ch_can_hat+.png](/images/Components/2ch_can_hat+.png)

---

[Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)

With 2–3× the speed of the previous generation, and featuring silicon designed in‑house for the best possible performance, we’ve redefined the Raspberry Pi experience.

![raspberry_pi_5.png](/images/Components/Raspberry_Pi_5.png)

---

[eSUN PLA+](https://www.esun3d.com/pla-pro-product/?gad_source=1&gclid=Cj0KCQjwhMq-BhCFARIsAGvo0KeC-QWRS4DnIqkqa8veTYXWu-kzXJ9L_G0ItWB_pjbDCn08X7ATh8IaAqrwEALw_wcB) for prototyping parts.

eSUN PLA+ filament is modified based on PLA material, easy to print. In addition, PLA plus improves the toughness and layer adherence.

![esun_pla+](/images/Components/eSUN_PLA+.png)

---

## Mechanical Assembly

### SolidWorks Modeling
- Design the robot structure in **SolidWorks**.
- Ensure all mechanical components, including chassis, joints, actuators, and sensors, are properly positioned.
- Maintain proper constraints and tolerances for assembly.

![full_otto_cad.jpg](/images/CAD/Full_Otto_CAD.jpg)

--- 

### Exporting STL for URDF

- Carefully group related parts into **subassemblies** that move together to ensure proper functionality in simulation.
- Identify **rigid components** that can be combined into a single STL and **moving components** that require separate STL files.
- In **SolidWorks**, define the correct **mate constraints** to simulate real-world motion.
- Steps for exporting STL files:
  1. Open the **subassembly** in **SolidWorks**.
  2. Click **File → Save As**.
  3. Select **STL (.stl)** as the file format.
  4. Choose **Fine** resolution for improved mesh quality.
  5. Ensure units are set to **meters** to match URDF scale.
  6. Save the file and validate the STL integrity in **MeshLab** or **Blender**.
- Use **xacro** to structure URDF files for better modularity.
- Maintain a **consistent naming convention** for STL files to simplify the URDF structure.

![all_sub_assem.jpg](/images/CAD/All_subassembly.jpg)

---

## CyberGear Motor CANBus Configuration

### CANBus Network Setup
- Ensure that **CAN interfaces (CAN0, CAN1)** are enabled on the robot’s controller.
- Use a **USB-to-CAN adapter** if using a PC for debugging.

### Configuration Steps
1. **Baud Rate Configuration:**
   - The default baud rate for CyberGear motors is **500Kbps**.
   - Adjust using `candump` and `cansend` if necessary.

2. **Set Motor ID:**
   ```bash
   cansend can0 0x000#02.00.01.ID
   ```
   Replace `ID` with the desired motor ID.

3. **Check Motor Status:**
   ```bash
   candump can0
   ```
   - Ensure no **bus errors**.
   - Look for correct **heartbeat messages**.

4. **Enable Torque Mode:**
   ```bash
   cansend can0 0x000#02.00.02.01
   ```
   - The motor should respond with an **ACK message**.
   - Verify by checking the **status byte** in return messages.

5. **Set Target Speed or Position:**
   ```bash
   cansend can0 0x000#02.00.03.[speed_value]
   ```
   Replace `[speed_value]` with desired RPM.

6. **Disable Motor:**
   ```bash
   cansend can0 0x000#02.00.02.00
   ```

---

## Basic Posture of the Robot

### Default Position

---------->*this section is for future parameters.*<----------

- **Standing height:** `X` cm
- **Wheelbase:** `Y` cm
- **Joint angles:**
  - **Shoulder:** `0°`
  - **Elbow:** `90°`
  - **Wrist:** `0°`
- **Resting position:** Arms folded at sides

### Movement Configurations
- **Forward movement:** Controlled by Hub motor.
- **Joint movement:** Controlled by CyberGear motor commands.
- **Balancing:** Ensure **IMU feedback** is active.

![example_otto_posture.gif](/images/Video/Example_Otto_posture.gif)

---

## Material Specifications

---------->*this section is for future parameters.*<----------

| Component          | Material        | Properties                 |
|-------------------|---------------|---------------------------|
| Chassis Frame    | Aluminum 6061 | Lightweight, corrosion-resistant |
| Joints           | Carbon Fiber  | High strength-to-weight ratio |
| Wheels           | TPU Rubber    | Shock absorption, durability |
| Body Panels      | ABS Plastic   | High impact resistance, lightweight |

### Notes on Material Selection
- **Aluminum 6061**: Used for **frame rigidity** and **lightweight properties**.
- **Carbon Fiber**: Applied in **high-stress areas** to improve strength.
- **ABS Plastic**: Used in **outer shell** for cost efficiency and impact protection.
- **TPU Rubber**: Enhances **wheel traction** and improves longevity.

---

For additional support, please refer to the official [**CyberGear CANBus documentation**](https://github.com/belovictor/cybergear-docs/blob/main/instructionmanual/instructionmanual.md) and [**ROS2 URDF tutorials**](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html).