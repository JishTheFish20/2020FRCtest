/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelOfFourtune extends SubsystemBase {
  /**
   * Creates a new WheelOfFourtune.
   */

  private static WheelOfFourtune WOF;

  private CANSparkMax wofMotor;
  private CANEncoder wofEncoder;

  I2C.Port i2cPort = I2C.Port.kOnboard;

  ColorSensorV3 colorsSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch colorMatch = new ColorMatch();

  Color blueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  Color greenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  Color redTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  Color yellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  String colorString;
  String gameData;
  String color = "";

  public WheelOfFourtune() {
    colorMatch.addColorMatch(blueTarget);
    colorMatch.addColorMatch(greenTarget);
    colorMatch.addColorMatch(redTarget);
    colorMatch.addColorMatch(yellowTarget);

    wofMotor = new CANSparkMax(7, MotorType.kBrushless);
    wofEncoder = wofMotor.getEncoder();
  }

  public void RotateTo(int distance, double power){
    if(getDistance() <= distance){
      wofMotor.set(power);
    }else{
      wofMotor.stopMotor();
    }
  }

  public double getDistance(){
    return wofEncoder.getPosition();
  }


  @Override
  public void periodic() {
 
    
    final Color detectedColor = colorsSensor.getColor();
    final ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
    final double IR = colorsSensor.getIR();

    final int color = 1;

    gameData = DriverStation.getInstance().getGameSpecificMessage();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putString("DetectedColor", gameData);
    //SmartDashboard.putNumber("color", gameData.charAt(0));
  //   // This method will be called once per scheduler run
  }
}
