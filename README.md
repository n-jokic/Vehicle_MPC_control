# Autonomous Vehicle Control with Model Predictive Control

This repository implements two Model Predictive Control (MPC) approaches for controlling an active front steering system in autonomous vehicles. The methodologies are based on the principles discussed in the following scientific papers:

## Implemented Papers

### Paper 1: Trajectory Following MPC for Autonomous Vehicles

- **Authors:** Paolo Falcone, Francesco Borrelli, Jahan Asgari, Hongtei Eric Tseng, and Davor Hrovat, Fellow, IEEE
- **Abstract:** This paper introduces a Model Predictive Control (MPC) approach for active front steering in autonomous vehicles. The MPC controller computes the front steering angle to follow a known trajectory on slippery roads, optimizing for the highest possible entry speed. Two approaches with different computational complexities are presented - one using a nonlinear 10 DOF vehicle model and the other based on successive online linearization. The README discusses computational complexity, performance, and includes simulations and experimental tests up to 21 m/s on icy roads.
- **Index Terms:** Active steering, autonomous vehicles, model predictive control, nonlinear optimization, vehicle dynamics control, vehicle stability.

### Paper 2: Path Following MPC with Steering and Braking

- **Authors:** Paolo Falcone, H. Eric Tseng, Francesco Borrelli, Jahan Asgari, and Davor Hrovat
- **Abstract:** This paper proposes a path following Model Predictive Control (MPC) scheme using both steering and braking for obstacle avoidance maneuvers. The control objective is to track a desired path through a combined use of braking and steering. The scheme relies on the Nonlinear MPC (NMPC) formulation used in [F. Borrelli, et al., MPC-based approach to active steering for autonomous vehicle systems] and [P. Falcone, et al., Predictive active steering control for autonomous vehicle systems]. Two approaches are derived using NMPC formulation, with the first relying on a full tenth-order vehicle model and the second based on a simplified bicycle model. The README discusses the effectiveness of both approaches through simulations and experiments.
- **Keywords:** Vehicle dynamics control, model predictive control, autonomous vehicles.

## Getting Started

### Prerequisites

- MATLAB 2023b
- Simulink

### Installation

1. Clone this repository to your local machine:

    ```bash
    git clone https://github.com/your-username/your-repo.git
    cd your-repo
    ```

2. Open MATLAB and navigate to the cloned repository.

3. Open the `main.m` script in MATLAB.

### Usage

The primary functionality is encapsulated in the `main.m` script. Run simulations and generate figures based on the implemented models from the scientific papers. To do so:

1. Open MATLAB and navigate to the project directory.

2. Open the `main.m` script.

3. Run the script in MATLAB. This will execute simulations and generate figures showcasing the results.

4. Explore the generated figures and simulation results within MATLAB.

Feel free to customize the `main.m` script or explore other scripts within the repository.

## Simulation and Implementation Details

The project involves two vehicle models and Model Predictive Control (MPC) development:

1. **Nonlinear 10 DOF Vehicle Model:**
   - Detailed model capturing vehicle dynamics intricacies.
   - Simulation foundation for understanding vehicle behavior under various conditions.

2. **Simplified 6 DOF Vehicle Model (Bicycle Model):**
   - Simplified model reducing computational complexity.
   - MPC controllers developed for trajectory following.

3. **MPC Controllers:**
   - MATLAB functions used for MPC controllers solving non-linear optimization problems.
   - Controllers use information from vehicle models to compute optimal control inputs, focusing on the front steering angle, for trajectory following.

4. **Friction Assumptions:**
   - Main assumption related to road friction, considering the worst-case scenario of driving on icy roads.
   - Assumption ensures robustness in challenging conditions.

## Report

The detailed report documenting the project, including theoretical background, methodology, and simulation results, is located in the "LaTeX" subfolder. The report is generated in Serbian to provide comprehensive documentation of the work.

To access the report:

1. Navigate to the "LaTeX" subfolder within the project directory.
2. Open the LaTeX project files using your preferred LaTeX editor.
3. Compile the LaTeX document to generate the PDF report.

The report aims to provide a thorough understanding of the implemented models, control strategies, simulation results, and their implications in the context of autonomous vehicle control.

## Contributing

Contributions to this project are welcome! To contribute:

1. Fork the project to your GitHub account.
2. Create a new branch for your changes: `git checkout -b feature/your-feature-name`.
3. Make your modifications and commit changes: `git commit -m "Description of your changes"`.
4. Push your changes to the new branch: `git push origin feature/your-feature-name`.
5. Open a pull request, explaining the changes and benefits to the project.

Ensure proper credit to the original authors and respect the MIT License terms when making contributions.

Thank you for your interest in contributing!

## License

This project is distributed under the [MIT License](LICENSE). Feel free to use, modify, and distribute the code as per the terms of the MIT License.
