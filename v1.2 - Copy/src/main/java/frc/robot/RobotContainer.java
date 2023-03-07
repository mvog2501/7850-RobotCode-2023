// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.subsystems.RobotArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ArmUpCmd;
import frc.robot.commands.ArmDownCmd;
import frc.robot.commands.ArmInCmd;
import frc.robot.commands.ArmOutCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroHeadingCmd;
import frc.robot.Constants.OperatorConstants.JoystickConstants;
// import frc.robot.Constants.OperatorConstants.LeftStickButtonPort;
// import frc.robot.Constants.OperatorConstants.RightStickButtonPort;
// import frc.robot.Constants.OperatorConstants.RightJoystickAxes;
// import frc.robot.Constants.OperatorConstants.LeftJoystickAxes;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final RobotArmSubsystem robotArmSubsystem = new RobotArmSubsystem();

  private final Joystick rightStick = new Joystick(JoystickConstants.rightStickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> rightStick.getRawAxis(JoystickConstants.kDriverXAxis),
              () -> -rightStick.getRawAxis(JoystickConstants.kDriverYAxis),
              () -> -rightStick.getRawAxis(JoystickConstants.kDriverRotAxis),
              () -> rightStick.getRawButton(JoystickConstants.kDriverFieldOrientedButtonIdx))); // Defaults to field reference
  
      // RobotArmSubsystem RobotArmSubsystem;
      // robotArmSubsystem.setDefaultCommand(new ArmOpenCmd(
      //   robotArmSubsystem,
      //   () -> rightStick.getRawButton(JoystickConstants.kButton12)));
    // Configure the trigger bindings
    configureButtonBindings();

  }


  private void configureButtonBindings() {
  
    new JoystickButton(rightStick, JoystickConstants.kDriverZeroButton).onTrue(new ZeroHeadingCmd(swerveSubsystem));


    //enables arm motors to go up
    new JoystickButton(rightStick, 6).onTrue(new ArmUpCmd(robotArmSubsystem));
    

    //enables extension arm motor
    new JoystickButton(rightStick, 5).onTrue(new ArmOutCmd(robotArmSubsystem));

    //enables retraction arm motor
    new JoystickButton(rightStick, 7).onTrue(new ArmInCmd(robotArmSubsystem));

    //enables arm motors to go down
    new JoystickButton(rightStick, 8).onTrue(new ArmDownCmd(robotArmSubsystem));
    
  }

 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  //}
}