// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.List;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.OperateDrive;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.ShootUtil;
import frc.robot.commands.OperateShoot;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	private final DriveUtil driveUtil = new DriveUtil();
	private final ShootUtil shootUtil = new ShootUtil();

	private final OperateDrive operateDrive = new OperateDrive(driveUtil);
	private final OperateShoot OperateShoot = new OperateShoot(shootUtil);

	private static XboxController driver;
	// private static XboxController operator;

	/**
	 * Added a new object - JoystickButton
	 * This one is used to Toggle the Climb Arm out and back.
	 */

	public static SendableChooser<Byte> driveType;
	public static SendableChooser<Byte> noobMode;
	public static SendableChooser<String> teamColorChooser;
	public final static Byte arcade = 0;
	public final static Byte tank = 1;
	public final static Byte curvature = 2;

	private SendableChooser<Command> autoChooser = new SendableChooser<>();


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		driver = new XboxController(Constants.XBOX_DRIVER);

		driveType = new SendableChooser<>();
		driveType.setDefaultOption("Curvature", curvature);
		driveType.addOption("Tank", tank);
		driveType.addOption("Arcade", arcade);
		SmartDashboard.putData("Drive Type", driveType);

		// Configure the button bindings
		configureButtonBindings();
		configureDefaultCommands();

		SmartDashboard.putData("Autonomous Command", autoChooser);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return autoChooser.getSelected();
	}

	private void configureDefaultCommands() {
		driveUtil.setDefaultCommand(operateDrive);
		shootUtil.setDefaultCommand(OperateShoot);
	}

	public static double getDriverLeftXboxX() {
		return driver.getLeftX();
	}

	public static double getDriverLeftXboxY() {
		return driver.getLeftY();
	}

	public static double getDriverRightXboxX() {
		return driver.getRightX();
	}

	public static double getDriverRightXboxY() {
		return driver.getRightY();
	}

	public static double getDriverLeftXboxTrigger() {
		return driver.getLeftTriggerAxis();
	}

	public static double getDriverRightXboxTrigger() {
		return driver.getRightTriggerAxis();
	}

	public static boolean getDriverAButton() {
		return driver.getAButton();
	}

	public static boolean getDriverBButton() {
		return driver.getBButton();
	}

	public static boolean getDriverXButton() {
		return driver.getXButton();
	}

	public static boolean getDriverYButton() {
		return driver.getYButton();
	}

	public static boolean getDriverLeftBumper() {
		return driver.getLeftBumper();
	}

	public static boolean getDriverRightBumper() {
		return driver.getRightBumper();
	}

	public static boolean getDriverLeftStickButton() {
		return driver.getLeftStickButton();
	}

	public static boolean getDriverRightStickButton() {
		return driver.getRightStickButton();
	}
}