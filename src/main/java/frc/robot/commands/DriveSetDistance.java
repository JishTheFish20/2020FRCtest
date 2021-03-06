/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveSetDistance extends CommandBase {
  /**
   * Creates a new DriveSetDistance.
   */

   private DriveTrain driveTrain;
   private double maxPower;
   private int distance;
   
  public DriveSetDistance(DriveTrain subsystem, double maxPower, int distance) {
    driveTrain = subsystem;
    addRequirements(driveTrain);
    this.maxPower = maxPower;
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //driveTrain.DriveSetDistance(power, turnSpeed, distance);
    driveTrain.DriveSetDistancePID(distance, maxPower);
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
