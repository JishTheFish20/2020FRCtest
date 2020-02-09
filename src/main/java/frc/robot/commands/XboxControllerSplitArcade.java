/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class XboxControllerSplitArcade extends CommandBase {

  private DriveTrain driveTrain;
  private XboxController joy;

  /**
   * Creates a new XboxControllerSplitArcade.
   */
  public XboxControllerSplitArcade(DriveTrain subsystem, XboxController joy) {
    driveTrain = subsystem;
    addRequirements(driveTrain);
    this.joy = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joy.getTriggerAxis(Hand.kRight) > 0){
    driveTrain.ControllerSplitArcade(joy);
    }else{
      driveTrain.stop();
    }
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
