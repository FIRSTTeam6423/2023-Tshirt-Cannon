// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootUtil;
import frc.robot.RobotContainer;

public class OperateShoot extends CommandBase {
  /** Creates a new OperateShoot. */

  private ShootUtil shootUtil;
  public OperateShoot(ShootUtil su) {
    this.shootUtil = su;
    addRequirements(this.shootUtil);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shootUtil.OperateCannon(RobotContainer.getDriverAButton());
    shootUtil.MoveTurret(RobotContainer.getDriverRightXboxY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
