/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
  /**
   * Creates a new ArcadeDrive.
   */

  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_velocity;
  private final DoubleSupplier m_heading;
  private final DoubleSupplier m_throttle;
  private final DoubleSupplier m_turnRate;
  
  public ArcadeDrive(DriveSubsystem subsystem, DoubleSupplier velocity, DoubleSupplier heading, DoubleSupplier throttle, DoubleSupplier turnRate) {
    m_drive = subsystem;
    m_velocity = velocity;
    m_heading = heading;
    m_throttle = throttle;
    m_turnRate = turnRate;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_velocity.getAsDouble() * (2 / (m_throttle.getAsDouble() + 3)), m_heading.getAsDouble() * (2 / (-m_turnRate.getAsDouble() + 3)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
