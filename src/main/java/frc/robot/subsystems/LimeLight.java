/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  /**
   * Creates a new LemonLime
   */

  private static LimeLight limeLight;

  public NetworkTable table;
  public NetworkTableEntry pipeline, camMode, ledMode, tx, ty, ta, tv;

  public double distance, cameraAngle = 16, cameraHeight = 38, targetHeight = 88;

  public double horizontalSetpoint, verticalSetpoint;

  private double Kp = 0.014, Ki = 0.075, Kd, integral, derivative;

  private double Kpy = 0.007, Kiy = 0.005, Kdy = 0, integralY, derivativeY;

  private double error, prevError, prevErrory, errory, steeringAdjust, steeringAdjustV;

  private double min, max, leftDriveCalc, rightDriveCalc;

  public double leftPower, rightPower;



  public LimeLight() {
    horizontalSetpoint = 0;
  }

  public static LimeLight getInstance(){
		if(limeLight == null) limeLight = new LimeLight();
    	return limeLight;
    }


  public double getHorizontalOffset() {
    return tx.getDouble(0.0);
  }

  public double getVerticalOffset() {
    return ty.getDouble(0.0);
  }

  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  public boolean validTarget() {
    if (tv.getDouble(0.0) == 1.0) {
      return true;
    } else {
      return false;
    }
  }

  // public double getCoorX() {
  //   nx = (1 / 160) * (getHorizontalOffset() - 159.5);
  //   return nx;
  // }

  public double GetDistance() {

    final double angle = cameraAngle + getVerticalOffset();
    distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));
    return distance;
  }

  public void seekTarget() {
    //if(validTarget() == true){
    error = getHorizontalOffset();
    errory = 150 - GetDistance();
    

    if(Math.abs(error) < 12) {
      integral += (error * 0.02);
    }
    
    if(Math.abs(error) < 0.45){
      integral = 0;
    }

    if(Math.abs(errory) < 40) {
      integralY += (errory * 0.02);
    }
    
    if(Math.abs(errory) < 1){
      integralY = 0;
    }

    
    
    //integralY += -(errory * 0.02);

    derivative = -(error - prevError) / 0.2;
    derivativeY = (errory - prevError) / 0.2;

    if (getHorizontalOffset() > horizontalSetpoint) {
      steeringAdjust = (Kp * error + Ki * integral) - min;
    }

    else if (getHorizontalOffset() < horizontalSetpoint) {
      steeringAdjust = (Kp * error + Ki * integral) + min;
    }

    steeringAdjustV = (Kpy * errory + Kiy * integralY);

    rightDriveCalc = steeringAdjust + steeringAdjustV;
    leftDriveCalc = -steeringAdjust + steeringAdjustV;
// }else{
//   rightDriveCalc = 0;
//   leftDriveCalc = 0;
// }

if (validTarget() == false) {
  rightDriveCalc = 0;
  leftDriveCalc = 0;
} 
    
    
    
    //leftPower = leftDriveCalc; //+ -steeringAdjustV;
    //rightPower = rightDriveCalc; //+ steeringAdjustV;

    prevError = error;
    prevErrory = errory;

}

  public double getTankDriveTrackingPowerLeft(){
    return leftDriveCalc;
  }

  public double getTankDriveTrackingPowerRight(){
    return rightDriveCalc;
  }

  public void getLimeLightSmartDashboard(){
    SmartDashboard.putNumber("HorizontalOffset", getHorizontalOffset());
    SmartDashboard.putNumber("VerticalOffset", getVerticalOffset());
    SmartDashboard.putNumber("TargetArea", getTargetArea());
    SmartDashboard.putBoolean("ValidTarget", validTarget());
    SmartDashboard.putNumber("IntegralX", integral);
    SmartDashboard.putNumber("IntegralY", integralY);
    SmartDashboard.putNumber("distance", GetDistance());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");

    getLimeLightSmartDashboard();
  }
}
