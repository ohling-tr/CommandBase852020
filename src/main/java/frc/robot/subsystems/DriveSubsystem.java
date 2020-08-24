/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import java.net.CacheRequest;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

  private CANSparkMax m_leftController1;
  private CANSparkMax m_leftController2;
  private CANSparkMax m_rightController1;
  private CANSparkMax m_rightController2;
  private SpeedControllerGroup m_leftControlGroup;
  private SpeedControllerGroup m_rightControlGroup;
  private DifferentialDrive m_diffDrive;

  private CANEncoder m_leftEncoder1;
  private CANEncoder m_rightEncoder1;

  private ADXRS450_Gyro m_gyro;

  public DriveSubsystem() {

    m_leftController1 = new CANSparkMax(DriveConstants.kLEFT_MOTOR_1_PORT, MotorType.kBrushless);
    m_leftController2 = new CANSparkMax(DriveConstants.kLEFT_MOTOR_2_PORT, MotorType.kBrushless);
    m_rightController1 = new CANSparkMax(DriveConstants.kRIGHT_MOTOR_1_PORT, MotorType.kBrushless);
    m_rightController2 = new CANSparkMax(DriveConstants.kRIGHT_MOTOR_2_PORT, MotorType.kBrushless);

    m_leftController1.restoreFactoryDefaults();
    m_leftController2.restoreFactoryDefaults();
    m_rightController1.restoreFactoryDefaults();
    m_rightController2.restoreFactoryDefaults();

    m_leftController1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_leftController2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightController1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightController2.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_leftController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_leftController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
  
    m_leftController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMIT);
    m_leftController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMIT);
    m_rightController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMIT);
    m_rightController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMIT);

    m_leftControlGroup = new SpeedControllerGroup(m_leftController1, m_leftController2);
    m_rightControlGroup = new SpeedControllerGroup(m_rightController1, m_rightController2);
    m_diffDrive = new DifferentialDrive(m_leftControlGroup, m_rightControlGroup);

    m_leftEncoder1 = m_leftController1.getEncoder();
    //m_leftEncoder1.setPositionConversionFactor(DriveConstants.kENCODER_DISTANCE_PER_PULSE_M);
    m_rightEncoder1 = m_rightController1.getEncoder();
    //m_rightEncoder1.setPositionConversionFactor(DriveConstants.kENCODER_DISTANCE_PER_PULSE_M);
    resetEncoders();
    
    m_gyro = new ADXRS450_Gyro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro....:", getAngle());
    SmartDashboard.putNumber("LeftEncdr:", getLeftEncoder());
    SmartDashboard.putNumber("RightEncdr:", getRightEncoder());
  }

  public void arcadeDrive(final double velocity, final double heading) {
    final double dDriveInvert = -1;
    m_diffDrive.arcadeDrive(velocity * dDriveInvert, heading);
  }

  private double getAngle() {
    return m_gyro.getAngle();
  }

  public void resetDrive() {
    resetAngle();
    resetEncoders();
  }

  private void resetAngle() {
    m_gyro.reset();
  }

  private double getLeftEncoder() {
    return m_leftEncoder1.getPosition() * DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
  }

  private double getRightEncoder() {
    return m_rightEncoder1.getPosition() * DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
  }

  private void resetEncoders() {
    m_leftEncoder1.setPosition(0);
    m_rightEncoder1.setPosition(0);
  }
}
