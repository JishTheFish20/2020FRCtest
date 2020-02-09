/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class LimeLightTracking extends CommandBase {
  /**
   * Creates a new LimeLightTracking.
   */

    //private DriveTrain driveTrain;
    private LimeLight limeLight;
    private DriveTrain driveTrain;
    private XboxController joy;

  public LimeLightTracking(LimeLight subsystem, DriveTrain subsystem2, XboxController joy) {
    limeLight = subsystem;
    driveTrain = subsystem2;
    addRequirements(limeLight);
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
    if(joy.getXButton()) {
      limeLight.seekTarget();
      driveTrain.LimelightTracking(limeLight.getTankDriveTrackingPowerLeft(), limeLight.getTankDriveTrackingPowerRight());
    }
    // }else{
    //   driveTrain.stop();
    // }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //driveTrain.LimelightTracking(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
