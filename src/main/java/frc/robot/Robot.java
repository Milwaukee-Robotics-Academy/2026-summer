// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
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
@Logged
public class Robot extends TimedRobot {
    private static final double SPEED_LIMIT = 0.6;
    private final DifferentialDrive m_robotDrive;
    private final XboxController driverController = new XboxController(0);
  
    private final SparkMax m_leftMotor = new SparkMax(1 , MotorType.kBrushless);
    private final SparkMax m_rightMotor = new SparkMax(2 , MotorType.kBrushless);
    private final SparkMax m_ballMotor = new SparkMax(3 , MotorType.kBrushless);
    private final SparkMaxConfig defaultConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightDriveConfig = new SparkMaxConfig();
    private final SparkMaxConfig ballMotorConfig = new SparkMaxConfig();

        // 2. Define physical constants
    private final double kGearRatio = 10.71; // 10.71 motor rotations per 1 wheel rotation
    private final double kWheelDiameterInches = 4.0;
    
    // 3. Calculate distance traveled per 1 motor revolution
    // Distance = (Wheel Circumference) / Gear Ratio
    private final double kPositionConversionFactor = (Math.PI * kWheelDiameterInches) / kGearRatio;

  
    /** Called once at the beginning of the robot program. */
    public Robot() {
  
      defaultConfig
          .inverted(false)
          .idleMode(IdleMode.kBrake)
          .openLoopRampRate(0.5)
          .voltageCompensation(12.0)
          .smartCurrentLimit(30);
  
      // We need to invert one side of the drivetrain so that positive voltages
      // result in both sides moving forward. Depending on how your robot's
      // gearbox is constructed, you might have to invert the left side instead.
      rightDriveConfig.apply(defaultConfig)
          .inverted(true);
      
      ballMotorConfig.apply(defaultConfig)
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .openLoopRampRate(0.0)
          .smartCurrentLimit(60);
  
      m_leftMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_rightMotor.configure(rightDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_ballMotor.configure(ballMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  
      SendableRegistry.addChild(m_robotDrive, m_leftMotor);
      SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    }
  
    @Override
    public void robotInit() {
      // This function is run when the robot is first started up and should be used for any
      // initialization code.
    }
  
    @Override
    public void robotPeriodic() {
      // This function is called every robot packet, no matter the mode. Use this for items like
      // diagnostics that you want ran during disabled, autonomous, teleoperated and test.
      // This runs after the mode specific periodic functions, but before LiveWindow and
      // SmartDashboard integrated updating.
      updateSmartDashboard();
    }
    @Override
    public void teleopPeriodic() {
      // Drive with arcade style (use right stick to steer and left stick to drive)
      //m_robotDrive.arcadeDrive(-driverController.getLeftY(), -driverController.getRightX());
      //m_robotDrive.tankDrive(-driverController.getLeftY(),-driverController.getRightY());
      m_robotDrive.arcadeDrive(driverController.getRightTriggerAxis()-driverController.getLeftTriggerAxis()*SPEED_LIMIT, -driverController.getRightY()*SPEED_LIMIT);


    // Use the triggers to control the ball motor
    
    if (driverController.getRightTriggerAxis() > 0.5) {
      m_ballMotor.set(-0.6);
    } else if (driverController.getLeftTriggerAxis() > 0.5) {
      m_ballMotor.set(0.5);
    } else {
      m_ballMotor.set(0.0);
    }

  }

  @Override
  public void autonomousInit() {
    m_leftMotor.getEncoder().setPosition(0);
    m_rightMotor.getEncoder().setPosition(0);
  }

  @Override
  public void autonomousPeriodic() {
    // This function is called periodically during autonomous.


    double aveEncoderCount = (m_leftMotor.getEncoder().getPosition()+m_rightMotor.getEncoder().getPosition())/2;
    double distance = aveEncoderCount * kPositionConversionFactor; // Convert encoder counts to distance in inches

    SmartDashboard.putNumber("Autonomous/Distance", distance);

    if (distance < 12) {
      m_robotDrive.arcadeDrive(0.5, 0); // Drive forward at half speed
    } else {
      m_robotDrive.arcadeDrive(0, 0); // Stop the robot
    }

  }

  public void updateSmartDashboard() {
    updateDriverControllerValues();
    // This method can be used to send data to the SmartDashboard for debugging purposes
    SmartDashboard.putNumber("LeftMotor/Output", m_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("RightMotor/Output", m_rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("BallMotor/Output", m_ballMotor.getAppliedOutput());
    SmartDashboard.putNumber("LeftMotor/Temperature", m_leftMotor.getMotorTemperature());
    SmartDashboard.putNumber("RightMotor/Temperature", m_rightMotor.getMotorTemperature());  
    SmartDashboard.putNumber("BallMotor/Temperature", m_ballMotor.getMotorTemperature());  
    SmartDashboard.putNumber("LeftMotor/Current", m_leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("RightMotor/Current", m_rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("BallMotor/Current", m_ballMotor.getOutputCurrent());
    SmartDashboard.putNumber("LeftMotor/Voltage", m_leftMotor.getBusVoltage());
    SmartDashboard.putNumber("RightMotor/Voltage", m_rightMotor.getBusVoltage());
    SmartDashboard.putNumber("BallMotor/Voltage", m_ballMotor.getBusVoltage());
    SmartDashboard.putNumber("LeftMotor/RPM", m_leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("RightMotor/RPM", m_rightMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("BallMotor/RPM", m_ballMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("LeftMotor/Position", m_leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("RightMotor/Position", m_rightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("BallMotor/Position", m_ballMotor.getEncoder().getPosition());
    SmartDashboard.putData(m_robotDrive);

  }

  private void updateDriverControllerValues() {
    SmartDashboard.putNumber("DriverController/joysticks/Left Stick Y", driverController.getLeftY());
    SmartDashboard.putNumber("DriverController/joysticks/Left Stick X", driverController.getLeftX());
    SmartDashboard.putNumber("DriverController/joysticks/Right Stick Y", driverController.getRightY());
    SmartDashboard.putNumber("DriverController/joysticks/Right Stick X", driverController.getRightX());
    SmartDashboard.putNumber("DriverController/triggers/Left Trigger", driverController.getLeftTriggerAxis());
    SmartDashboard.putNumber("DriverController/triggers/Right Trigger", driverController.getRightTriggerAxis()); 
    SmartDashboard.putBoolean("DriverController/buttons/A Button", driverController.getAButton());
    SmartDashboard.putBoolean("DriverController/buttons/B Button", driverController.getBButton());
    SmartDashboard.putBoolean("DriverController/buttons/X Button", driverController.getXButton());
    SmartDashboard.putBoolean("DriverController/buttons/Y Button", driverController.getYButton());
    SmartDashboard.putBoolean("DriverController/buttons/Left Bumper", driverController.getLeftBumperButton());
    SmartDashboard.putBoolean("DriverController/buttons/Right Bumper", driverController.getRightBumperButton());
    SmartDashboard.putBoolean("DriverController/buttons/Back Button", driverController.getBackButton());
    SmartDashboard.putBoolean("DriverController/buttons/Start Button", driverController.getStartButton());
    SmartDashboard.putBoolean("DriverController/buttons/Left Stick Button", driverController.getLeftStickButton());
    SmartDashboard.putBoolean("DriverController/buttons/Right Stick Button", driverController.getRightStickButton());
    SmartDashboard.putNumber("DriverController/Pov", driverController.getPOV());
    SmartDashboard.putNumber("DriverController/Port", driverController.getPort());
    SmartDashboard.putNumber("DriverController/Axis Count", driverController.getAxisCount());
    SmartDashboard.putNumber("DriverController/Button Count", driverController.getButtonCount());
    // This method can be used to read values from the driver controller and update any necessary variables
  }

}
