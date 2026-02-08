// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
    private SparkFlex motor;
    private SparkFlexConfig Config;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public Climber()
    {
        motor = new SparkFlex(ClimerConstants.MOTOR_ID, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.get;
    }

}