// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// v1.1 test cmd

package frc.robot.commands;

import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class ArmOpenCmd extends CommandBase {
  
  private final GrabberSubsystem grabberSubsystem;

 
  /**
   * @param grabberSubsystem
   * @param d
   */
  public ArmOpenCmd (GrabberSubsystem grabberSubsystem, double d) {
    this.grabberSubsystem = grabberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grabberSubsystem);


  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    grabberSubsystem.testspeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  
}
