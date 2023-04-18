// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ShootUtil extends SubsystemBase{
    private CANSparkMax turretMotor;
    private RelativeEncoder turretEncoder;

    private Solenoid shooter;
    private Compressor pcmCompressor;

    public ShootUtil(){
        shooter = new Solenoid(10, PneumaticsModuleType.CTREPCM, 0);
        pcmCompressor = new Compressor(10, PneumaticsModuleType.CTREPCM);
        pcmCompressor.enableDigital();
        turretMotor = new CANSparkMax(Constants.TURRET_MOTOR, MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        turretEncoder.setPosition(0);

        turretEncoder.setPositionConversionFactor(0.1);
    }

    public void ShootTshirt(boolean button){
        shooter.set(button);
    }

    public void MoveTurret(double joystickInput){
        if(turretEncoder.getPosition() < Constants.TURRET_UPPER_LIMIT &&
            turretEncoder.getPosition() > Constants.TURRET_LOWER_LIMIT){
            turretMotor.set(MathUtil.clamp(joystickInput, 
            -0.4, 0.4));
        } else if(turretEncoder.getPosition() < Constants.TURRET_UPPER_LIMIT){ //too low
            turretMotor.set(MathUtil.clamp(joystickInput, 
            0, 0.4));
        } else{ //too high
            turretMotor.set(MathUtil.clamp(joystickInput, 
            -0.4, 0));
        }
    }
}
