package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotArmSubsystem;
import frc.robot.subsystems.RobotArmSubsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.OperatorConstants.ArmConstants;

public class ArmPIDCmd extends CommandBase {
    private final RobotArmSubsystem robotArmSubsystem;
    private final PIDController horizPID;
    private final PIDController vertPID;


    public ArmPIDCmd(RobotArmSubsystem robotArmSubsystem, double vertSetpoint, double horizSetpoint) {
        this.robotArmSubsystem = robotArmSubsystem;
        this.horizPID = new PIDController(ArmConstants.hP, ArmConstants.hI, ArmConstants.hD);
        this.vertPID = new PIDController(ArmConstants.vP, ArmConstants.vI, ArmConstants.vD);

        vertPID.setSetpoint(vertSetpoint);
        horizPID.setSetpoint(horizSetpoint);

        vertPID.setTolerance(1, 1);
        horizPID.setTolerance(1, 1);

        addRequirements(robotArmSubsystem);
    }

    @Override
    public void initialize() {
        vertPID.reset();
        horizPID.reset();
    }

    @Override
    public void execute() {
        double vertSpeed = vertPID.calculate(robotArmSubsystem.getVertEncoder());
        double horizSpeed = horizPID.calculate(robotArmSubsystem.getHorizEncoder());

        robotArmSubsystem.setVertMotors(vertSpeed);
        robotArmSubsystem.setHorizMotors(horizSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        robotArmSubsystem.stopVertMotors();
        robotArmSubsystem.stopHorizMotors();
    }
        
  

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return (vertPID.atSetpoint() && horizPID.atSetpoint());

  }

}
    

