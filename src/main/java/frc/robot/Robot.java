// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final XboxController driverController = new XboxController(0);

  private final SparkMax m_leftMotor = new SparkMax(1 , MotorType.kBrushless);
  private final SparkMax m_rightMotor = new SparkMax(2 , MotorType.kBrushless);
  private final SparkMax m_ballMotor = new SparkMax(3 , MotorType.kBrushless);
  private final SparkMaxConfig defaultConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightDriveConfig = new SparkMaxConfig();

  /** Called once at the beginning of the robot program. */
  public Robot() {

    defaultConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(80);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightDriveConfig.apply(defaultConfig)
        .inverted(true);

    m_leftMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(rightDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_ballMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade style (use right stick to steer and left stick to drive)
    m_robotDrive.arcadeDrive(-driverController.getLeftY(), -driverController.getLeftX());

    // Use the triggers to control the ball motor
    
    if (driverController.getRightTriggerAxis() > 0.5) {
      m_ballMotor.set(1.0);
    } else if (driverController.getLeftTriggerAxis() > 0.5) {
      m_ballMotor.set(-1.0);
    } else {
      m_ballMotor.set(0.0);
    }

  }

  public void updateSmartDashboard() {
    // This method can be used to send data to the SmartDashboard for debugging purposes
    SmartDashboard.putNumber("Left/Motor Output", m_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Right/Motor Output", m_rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("Ball/Motor Output", m_ballMotor.getAppliedOutput());
    SmartDashboard.putNumber("Left/Motor Temperature", m_leftMotor.getMotorTemperature());
    SmartDashboard.putNumber("Right/Motor Temperature", m_rightMotor.getMotorTemperature());  
    SmartDashboard.putNumber("Ball/Motor Temperature", m_ballMotor.getMotorTemperature());  
    SmartDashboard.putNumber("Left/Motor Current", m_leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right/Motor Current", m_rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Ball/Motor Current", m_ballMotor.getOutputCurrent());
    SmartDashboard.putNumber("Left/Motor Voltage", m_leftMotor.getBusVoltage());
    SmartDashboard.putNumber("Right/Motor Voltage", m_rightMotor.getBusVoltage());
    SmartDashboard.putNumber("Ball/Motor Voltage", m_ballMotor.getBusVoltage());
    SmartDashboard.putNumber("Left/Motor RPM", m_leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right/Motor RPM", m_rightMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Ball/Motor RPM", m_ballMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Left/Motor Position", m_leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right/Motor Position", m_rightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Ball/Motor Position", m_ballMotor.getEncoder().getPosition());
    SmartDashboard.putData(m_robotDrive);

  }

}
