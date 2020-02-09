/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIntake extends SubsystemBase {
  /**
   * Creates a new ShooterIntake.
   */

   private static ShooterIntake shooterIntake;

  private SpeedController motor;
  //private XboxController joy;

  public ShooterIntake() {
    motor = new VictorSP(0);
  }

  public static ShooterIntake getInstance() {
    if (shooterIntake == null)
      shooterIntake = new ShooterIntake();
    	return shooterIntake;
    }


  public void move(XboxController joy){
    if(joy.getAButton()){
      motor.set(1);
    }
    else if(joy.getBButton()){
      motor.set(-1);
    }else{
      motor.set(0);
    }
    //motor.set(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //move();
  }
}
