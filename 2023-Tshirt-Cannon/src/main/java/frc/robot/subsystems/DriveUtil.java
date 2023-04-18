// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveUtil extends SubsystemBase {
    private CANSparkMax leftPrimary, leftSecondary, rightPrimary, rightSecondary;

    private RelativeEncoder leftPrimaryEncoder, leftSecondaryEncoder, rightPrimaryEncoder, rightSecondaryEncoder;

    public double setpoint;
    // Change this to match the name of your camera
    final double CAMERA_HEIGHT = 0.8128;
    final double TARGET_HEIGHT = 1.2446;
    final double CAMERA_PITCH_RADIANS = 0;
    final double GOAL_RANGE_METERS = 0.3;

    // Drive controller
    private DifferentialDrive differentialDrive;

    public DriveUtil() {
        leftPrimary = new CANSparkMax(Constants.LEFT_PRIMARY, MotorType.kBrushless);
        leftSecondary = new CANSparkMax(Constants.LEFT_SECONDARY, MotorType.kBrushless);
        rightPrimary = new CANSparkMax(Constants.RIGHT_PRIMARY, MotorType.kBrushless);
        rightSecondary = new CANSparkMax(Constants.RIGHT_SECONDARY, MotorType.kBrushless);
        // testlinear = new CANSparkMax(9, MotorType.kBrushless);
        // testrotate = new CANSparkMax(10, MotorType.kBrushless);
        // rightPrimary.setInverted(false);

        //gyro.reset();

        // Set secondaries to follow primaries
        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);
        // Initialize DifferentialDrive controller
        differentialDrive = new DifferentialDrive(leftPrimary, rightPrimary);

        leftPrimaryEncoder = leftPrimary.getEncoder();
        leftSecondaryEncoder = leftSecondary.getEncoder();
        rightPrimaryEncoder = rightPrimary.getEncoder();
        rightSecondaryEncoder = rightSecondary.getEncoder();
        
        leftPrimaryEncoder.setPositionConversionFactor(4096);
        leftSecondaryEncoder.setPositionConversionFactor(4096);
        rightPrimaryEncoder.setPositionConversionFactor(4096);
        rightSecondaryEncoder.setPositionConversionFactor(4096);

        leftPrimary.setInverted(true);
        // leftPrimaryEncoder.setInverted(true);
        // rightPrimaryEncoder.setInverted(true);
        // leftSecondaryEncoder.setInverted(true);
        // rightSecondaryEncoder.setInverted(true);

        leftPrimaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);

        leftPrimary.setIdleMode(IdleMode.kCoast);
        rightPrimary.setIdleMode(IdleMode.kCoast);
        leftSecondary.setIdleMode(IdleMode.kCoast);
        rightSecondary.setIdleMode(IdleMode.kCoast);
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

    // public void testSwerve() {
    //     testlinear.setVoltage(0.1);
    //     testrotate.setVoltage(0.1);
    // }

    // public void stopSwerve() {
    //     testlinear.setVoltage(0);
    //     testrotate.setVoltage(0);
    // }

    public void tankDrive(double left, double right) {
        differentialDrive.tankDrive(left, right);
    }

    public void stopDistance() {
        differentialDrive.tankDrive(0, 0);
    }

    public double getPosition() {
        double sensorPosition = (leftPrimaryEncoder.getPosition() - rightPrimaryEncoder.getPosition()) / 2;

        return sensorPosition;
    }

    public boolean getMoving() {
        return leftPrimary.get() > 0.1 && rightSecondary.get() > 0.1;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftPrimaryEncoder.getVelocity(),rightPrimaryEncoder.getVelocity());
    }

    
    public void resetEncoders(){
        leftPrimaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);
    }

    public void drive(double leftPercentPower, double rightPercentPower) {
        leftPrimary.set(leftPercentPower);
        leftSecondary.set(leftPercentPower);
        rightPrimary.set(rightPercentPower);
        rightSecondary.set(rightPercentPower);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        //SmartDashboard.putString("Drive Type   ::  ", RobotContainer.driveType.getSelected().toString());
        //SmartDashboard.putString("Yaw   ::  ", Double.toString(yaw));
    }

}