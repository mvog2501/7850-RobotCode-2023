package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotArmSubsystem;;

public class ArmPIDCmd extends CommandBase{

    private final RobotArmSubsystem robotArmSubsystem;

 
  /**
   * @param robotArmSubsystem
   * @param d
   */
  public ArmPIDCmd (RobotArmSubsystem robotArmSubsystem) {
    this.robotArmSubsystem = robotArmSubsystem;
  


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotArmSubsystem);


  }


    private void addRequirements(RobotArmSubsystem robotArmSubsystem) {
      }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {

    robotArmSubsystem.stopVertMotors();

  }
  
  @Override
  public boolean isFinished() {

    return false;

  }
}
    

