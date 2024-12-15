# FRC Team 7419 - 2025 Reefscape Robot Code

## Overview

This repository contains the codebase the 2025 FRC Reefscape season.

## Development Guidelines

### Code Quality

- **Consistency**: Adhere to established coding standards to ensure uniformity across the codebase.
- **Readability**: Write clear and understandable code to facilitate maintenance and collaboration.
- **Documentation**: Include comments where necessary to explain complex logic or decisions.

### Naming Conventions

- **Subsystems**: Use `UpperCamelCase`. For example, `DriveSubsystem`.
- **Commands**: Use descriptive `UpperCamelCase`. For example, `RunArmWithJoystick` or `DriveToWaypoint`.
- **Constants**:
  - In the constants file: Prefix with 'k' and use `UpperCamelCase`. For example, `kMaxSpeed`.
  - Within other files: Use `UPPER_SNAKE_CASE`. For example, `MAX_SPEED`.

Following these conventions enhances code clarity and maintainability.

## Build Process

We utilize [Spotless](https://github.com/diffplug/spotless) to maintain consistent code formatting. The build process automatically applies Spotless checks. Therefore, ensure you **build the project successfully before committing** to verify that all code adheres to the formatting standards.

Your PR will fail if not :(

## Branching Strategy

- **Branch Creation**: Create a new branch for each feature or bug fix. Avoid committing directly to the `master` branch.
- **Naming**: Use clear names that convey the purpose of the branch, enabling team members to easily understand its focus. For example, `drivetrain` or `intake`.
- **Committing Unfinished Work**: It's acceptable to commit unfinished code to your own branches to allow others to continue your work if necessary. Ensure that such commits are well-documented in Slack, clearly indicating the current status and any remaining tasks.

By following this strategy, we maintain a clean and organized commit history and avoid confusion.

## Visual Studio Code Workspace

A customized VS Code workspace configuration is provided to streamline development:

- **Workspace File**: The `.code-workspace` file in the root directory includes specific settings, launch configurations, and extension recommendations tailored for this project.
- **Setup**: To utilize this configuration, open the workspace file in VS Code by navigating to `File > Open Workspace...` and selecting the `.code-workspace` file.
- **Benefits**: This setup ensures a consistent development environment across the team, with predefined settings and tools.

## Additional Resources

- [FIRST Robotics Competition Documentation](https://docs.wpilib.org/)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/en/stable/)
- [PathPlanner Documentation](https://pathplanner.dev/home.html)
- [PhotonVision Documentation](https://docs.photonvision.org/en/latest/)
- [Limelight Documentation](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib)
- [REVLib Documentation](https://docs.revrobotics.com/brushless/revlib/revlib-overview)
- [Elastic Dashboard](https://frcdocs.wpi.edu/en/latest/docs/software/dashboards/elastic.html)
