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

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
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
  private double m_leftEncoderSign;
  private double m_rightEncoderSign;
  private double m_headingSign;

  private AHRS m_gyroK;

  private final DifferentialDriveOdometry m_driveOdometry;

  public DriveSubsystem() {

    /*
       public static final double kGEARBOX_REDUCTION = (50.0/12.0) * (60.0/14.0);
        public static final double kTIRE_SIZE_IN = 7.9;
        public static final double kTIRE_SIZE_M = Units.inchesToMeters(kTIRE_SIZE_IN);
        public static final int kPULSE_PER_ROTATION = 1;
        //public static final double kENCODER_DISTANCE_PER_PULSE_M = ((double) kPULSE_PER_ROTATION / kGEARBOX_REDUCTION) * (kTIRE_SIZE_M * Math.PI);
        public static final double kENCODER_DISTANCE_PER_PULSE_M = (k
    */
    System.out.format("Gearing.........: %f%n", DriveConstants.kGEARBOX_REDUCTION);
    System.out.format("Tire M..........: %f%n", DriveConstants.kTIRE_SIZE_M);
    System.out.format("PulseRot........: %d%n", DriveConstants.kPULSE_PER_ROTATION);
    System.out.format("DistancePerPulse: %f%n", DriveConstants.kENCODER_DISTANCE_PER_PULSE_M);

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
    m_leftControlGroup.setInverted(DriveConstants.kIS_DRIVE_INVERTED);
    m_rightControlGroup.setInverted(DriveConstants.kIS_DRIVE_INVERTED);
    m_diffDrive = new DifferentialDrive(m_leftControlGroup, m_rightControlGroup);
    
    /*
    m_leftController1.setInverted(DriveConstants.kIS_DRIVE_INVERTED);
    m_rightController1.setInverted(!DriveConstants.kIS_DRIVE_INVERTED);
    m_diffDrive = new DifferentialDrive(m_leftController1, m_rightController1);
    m_leftController2.follow(m_leftController1);
    m_rightController2.follow(m_rightController1);
    */
    
    if (DriveConstants.kIS_DRIVE_INVERTED) {
      m_leftEncoderSign = 1;
      m_rightEncoderSign = -1;
      m_headingSign = -1;
    } else {
      m_leftEncoderSign = -1;
      m_rightEncoderSign = 1;
      m_headingSign = 1;
    }
    


    m_leftEncoder1 = m_leftController1.getEncoder();
    m_rightEncoder1 = m_rightController1.getEncoder();
    resetEncoders();
    
    m_gyroK = new AHRS(SerialPort.Port.kMXP);

    m_driveOdometry = new DifferentialDriveOdometry(getRotation2dK());

    SendableRegistry.addLW(m_diffDrive, "Drive Base");
    SendableRegistry.addLW(m_gyroK, "NavX");
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_driveOdometry.update(getRotation2dK(), getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("GyroK", getAngleK());
    SmartDashboard.putNumber("LeftEncdr:", getLeftDistance());
    SmartDashboard.putNumber("RightEncdr:", getRightDistance());
    SmartDashboard.putNumber("Odo Y" , m_driveOdometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Odo X", m_driveOdometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Odo Deg", m_driveOdometry.getPoseMeters().getRotation().getDegrees());
  }

  public void arcadeDrive(final double velocity, final double heading) {
    m_diffDrive.arcadeDrive(velocity, m_headingSign * heading);
  }

  private double getAngleK() {
    return m_gyroK.getAngle();
  }

  private Rotation2d getRotation2dK() {
    // note the negation of the angle is required because the wpilib convention
    // uses left positive rotation while gyros read right positive
    return Rotation2d.fromDegrees(-getAngleK());
  }

  public void resetDrive() {
    resetAngle();
    resetEncoders();
  }

  private void resetAngle() {
    m_gyroK.zeroYaw();
    // need to add reset of odometry and encoders
  }

  private double getLeftDistance() {
    return m_leftEncoderSign * m_leftEncoder1.getPosition() * DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
  }

  private double getRightDistance() {
    return m_rightEncoderSign * m_rightEncoder1.getPosition() * DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
  }

  private void resetEncoders() {
    m_leftEncoder1.setPosition(0);
    m_rightEncoder1.setPosition(0);
  }
}
