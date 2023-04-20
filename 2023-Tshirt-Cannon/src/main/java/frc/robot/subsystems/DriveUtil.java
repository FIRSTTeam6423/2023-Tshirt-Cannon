// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveUtil extends SubsystemBase {
    private WPI_TalonSRX leftPrimary, leftSecondary, rightPrimary, rightSecondary;

    public double setpoint;
    // Change this to match the name of your camera
    final double CAMERA_HEIGHT = 0.8128;
    final double TARGET_HEIGHT = 1.2446;
    final double CAMERA_PITCH_RADIANS = 0;
    final double GOAL_RANGE_METERS = 0.3;


    // Drive controller
    private DifferentialDrive differentialDrive;

    public DriveUtil() {
        leftPrimary = new WPI_TalonSRX(Constants.LEFT_PRIMARY);
        leftSecondary = new WPI_TalonSRX(Constants.LEFT_SECONDARY);
        rightPrimary = new WPI_TalonSRX(Constants.RIGHT_PRIMARY);
        rightSecondary = new WPI_TalonSRX(Constants.RIGHT_SECONDARY);

        // Set secondaries to follow primaries
        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);
        // Initialize DifferentialDrive controller
        differentialDrive = new DifferentialDrive(leftPrimary, rightPrimary);

        leftPrimary.setInverted(true);
        // leftPrimaryEncoder.setInverted(true);
        // rightPrimaryEncoder.setInverted(true);
        // leftSecondaryEncoder.setInverted(true);
        // rightSecondaryEncoder.setInverted(true);
    }

    
    /**
     * Drive the robot based on the driveMode class parameter.
     * If in TANK mode, use leftX and rightX values.
     * If in ARCADE mode, use rightX and rightY values.
     * 
     * The DifferentialDrive class will square inputs for us.
     * Squaring inputs results in less sensitive inputs.
     * 
     * @param leftX  the left controller's X (forward-backward) value
     * @param leftY  the left controller's Y (left-right) value
     * @param rightX the right controller's X (forward-backward) value
     * @param rightY the right controller's Y (left-right) value
     */
    public void driveRobot() {
        // arcade drive
            if (RobotContainer.driveType.getSelected().equals(RobotContainer.arcade)) {
                // If we're in ARCADE mode, use arcadeDrive
                differentialDrive.arcadeDrive(RobotContainer.getDriverRightXboxX(),
                        -RobotContainer.getDriverRightXboxY() / 1.5);
            } else if (RobotContainer.driveType.getSelected().equals(RobotContainer.tank)) {
                // If we're in TANK mode, use tankDrive
                differentialDrive.tankDrive(-RobotContainer.getDriverLeftXboxY() / 2,
                        RobotContainer.getDriverRightXboxY() / 2);
            } else {
                // If we are in CURVATURE mode, use the curvature mode
                double rotation = RobotContainer.getDriverLeftXboxX();
                boolean isNegative = rotation < 0;

                rotation *= rotation;
                if (isNegative) {
                    rotation *= -1;
                }
                rotation *= 0.75;

                differentialDrive.curvatureDrive(rotation,
                        (-RobotContainer.getDriverLeftXboxTrigger() + RobotContainer.getDriverRightXboxTrigger()) / 2,
                        true);
            }
        

    }

    public void tankDrive(double left, double right) {
        differentialDrive.tankDrive(left, right);
    }

    public void stopDistance() {
        differentialDrive.tankDrive(0, 0);
    }


    public boolean getMoving() {
        return leftPrimary.get() > 0.1 && rightSecondary.get() > 0.1;
    }

    public void drive(double leftPercentPower, double rightPercentPower) {
        leftPrimary.set(leftPercentPower);
        leftSecondary.set(leftPercentPower);
        rightPrimary.set(rightPercentPower);
        rightSecondary.set(rightPercentPower);
    }

    @Override
    public void periodic() {
    }

}