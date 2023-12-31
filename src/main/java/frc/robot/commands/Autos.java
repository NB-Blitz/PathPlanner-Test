// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase swerveAuto(SwerveSubsystem subsystem) {
    return Commands.sequence(new SwerveCommand(subsystem, null, null, null, null, null));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
