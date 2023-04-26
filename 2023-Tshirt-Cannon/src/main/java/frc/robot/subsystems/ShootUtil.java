// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/** Add your docs here. */
public class ShootUtil extends SubsystemBase{
    private WPI_TalonSRX turretMotor;

    private DigitalOutput shoottest;
    private Solenoid shooter;
    private Compressor pcmCompressor;
    private boolean shot = false;
    private int ticks = 0;
    public ShootUtil(){
        shoottest = new DigitalOutput(0);
        // shooter = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);
        // pcmCompressor = new Compressor(10, PneumaticsModuleType.CTREPCM);
        // pcmCompressor.enableDigital();
        // turretMotor = new WPI_TalonSRX(Constants.TURRET_MOTOR);
        //shooter.setPulseDuration(Constants.SHOT_PULSE_DURATION);
    }

    public void OperateCannon(boolean button){
        if ((ticks / 50)>1) {
            shoottest.set(true);
            //ticks = 0;
            System.out.println("AAAAAAAAAAAAAAAAa");
        }
        shoottest.set(false);
        shoottest.pulse(0.00001);
        // if(!shot && button) {
        //     shot = true;
        //     shooter.startPulse();
        // } else if(!button) {
        //     shot = false;
        // }
        ticks +=1;
    }

    public void MoveTurret(double joystickInput){
        //turretMotor.set(MathUtil.clamp(joystickInput, -0.4, 0.4));
    }
}
