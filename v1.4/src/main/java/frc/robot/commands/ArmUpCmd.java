// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// v1.1 test cmd

package frc.robot.commands;

import frc.robot.subsystems.RobotArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;


/** An example command that uses an example subsystem. */
public class ArmUpCmd extends CommandBase {
  
  private final RobotArmSubsystem robotArmSubsystem;

 
  /**
   * @param robotArmSubsystem
   * @param d
   */
  public ArmUpCmd (RobotArmSubsystem robotArmSubsystem) {
    this.robotArmSubsystem = robotArmSubsystem;
  


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotArmSubsystem);


  }


private void addRequirements(RobotArmSubsystem robotArmSubsystem) {
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotArmSubsystem.testUp();
  }



  // Returns true when the command should end.
  @Override
  public void end(boolean interrupted) {

    robotArmSubsystem.stopVertMotors();

  }

  
}
