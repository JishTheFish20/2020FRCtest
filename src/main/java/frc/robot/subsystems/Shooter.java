/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private static Shooter shooter;

  private CANSparkMax rightShooter, leftShooter;

  private CANEncoder shooterEnc;

  private SpeedControllerGroup shooterAB;

  //private XboxController joy;

  private double power, RPM;

  private DifferentialDrive robot;

  private double Kp = 0.00035, Ki = 0.0001, Kf = 0, error, currentRPM, integral;

  public Shooter() {

    rightShooter = new CANSparkMax(Constants.A, MotorType.kBrushless);
    leftShooter = new CANSparkMax(Constants.B, MotorType.kBrushless);

    leftShooter.setInverted(true);
    rightShooter.setInverted(false);

    // rightShooter.setOpenLoopRampRate(0.5);
    // leftShooter.setOpenLoopRampRate(0.5);

    // rightShooter.enableVoltageCompensation(12.5);
    // leftShooter.enableVoltageCompensation(12.5);

    rightShooter.setSmartCurrentLimit(40);
    leftShooter.setSmartCurrentLimit(40);

    //rightShooter.set

    shooterEnc = rightShooter.getEncoder();

    //robot = new DifferentialDrive(shooterA, shooterB);
    shooterAB = new SpeedControllerGroup(rightShooter, leftShooter);

    //joy = new Joystick(Constants.DRIVER);

    SmartDashboard.putNumber("Setpoint", 0);

  }

  public static Shooter getInstance(){
		if(shooter == null) shooter = new Shooter();
    	return shooter;
    }

  public void move(XboxController joy){
    pid(SmartDashboard.getNumber("Setpoint", 5000));
   // robot.arcadeDrive(joy.getY(Hand.kRight), 0);
   //shooterAB.set(joy.getY(Hand.kRight));
   shooterAB.set(power);
  }

  // public void setSpeed(XboxController joy){
  //   //move();
  //   if(joy.getXButton()){
  //     power = power-1;
  //   }

  //   else if(joy.getBButton()){
  //     power = power+1;
  //   }

  //   else if(joy.getYButton()){
  //     power = 0;
  //   }

  //   else if(joy.getRawButton(5)){
  //     power = power+10;
  //   }

  //   else if(joy.getRawButton(6)){
  //     power = power-10;
  //   }
  // }

  public double getRPM(){
    return shooterEnc.getVelocity();
  }

  public double getLeftShooterTemp(){
    return leftShooter.getMotorTemperature();
  }

  public double getRightShooterTemp(){
    return rightShooter.getMotorTemperature();
  }

  public void pid(double setpoint){
    currentRPM = shooterEnc.getVelocity();
    error = (setpoint - currentRPM);

    if(Math.abs(error) < 500) {
      integral += (error*.02);
    } else {
      integral = 0;
    }
    
    Kf = (setpoint * 0.95) / (10500);

    power = (Kp*error + Ki*integral) + Kf;

    if(power < 0) {
      power = 0;
    }

    // if(getRPM() < 1000){
    //   integral = 0;
  }

  

  



  @Override
  public void periodic() {
    //pid(5000);

    // SmartDashboard.putNumber("RPM", getRPM());
    // SmartDashboard.putNumber("RightTemp", getRightShooterTemp());
    // SmartDashboard.putNumber("LeftTemp", getLeftShooterTemp());
    // SmartDashboard.putNumber("Integral", integral);
    // SmartDashboard.putNumber("Power", power);
    // SmartDashboard.putNumber("Kf", Kf);
  }
}
