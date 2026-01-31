// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.KrakenMotor;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Add your docs here. */
public class KrakenMotor{
public TalonFX motor;
public VelocityVoltage velocity;
public TalonFXConfiguration MotorPID;

public KrakenMotor(){
    motor = new TalonFX(1);
    velocity = new VelocityVoltage(0);
    MotorPID = new TalonFXConfiguration();

    MotorPID.Slot0.kP = 0.1;
    MotorPID.Slot0.kV = 0.12;

    motor.getConfigurator().apply(MotorPID);
}
private static KrakenMotor instance;

public static KrakenMotor getInstance(){
    if (instance == null){
        instance = new KrakenMotor();
    }
    return instance;
}
}
