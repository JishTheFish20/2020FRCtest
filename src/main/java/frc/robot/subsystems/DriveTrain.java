/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import com.kauailabs.navx.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  private static DriveTrain driveTrain;

  //private String trajectoryJSON = "paths/YourPath.wpilib.json";

  private static int[] DTL_IDs = { //Drive Train Left
    Constants.DTL_FRONT, Constants.DTL_BACK
};

  private static int[] DTR_IDs = { //Drive Train Left
    Constants.DTR_FRONT, Constants.DTR_BACK
};

  //  private SpeedController FrontLeft;
  //  private SpeedController BackLeft;
  //  private SpeedController FrontRight;
  //  private SpeedController BackRight;

   private CANSparkMax FrontLeft, BackLeft, FrontRight, BackRight;

   private SpeedControllerGroup RightDrive;
   private SpeedControllerGroup LeftDrive;

   private CANEncoder FrontRightEnc, FrontLeftEnc;

   private CANEncoder BackRightEnc, BackLeftEnc; 

   private double x,y;

   private double Kp,Ki,Kd, error, setPosition, integral, power;

   private double Kpa, Kia, Kda, errora, setPositiona, integrala, powera;

   private AHRS navx;

   private XboxController xbox;

   private DifferentialDrive robot;

   private DifferentialDriveOdometry odometry;

   private boolean gotToSetDistance, gotToSetAngle;
   
  public DriveTrain() {

    Kp = 0.025;
    Ki = 0.15;

    Kpa = 0.022;
    Kia = 0.08;

    FrontLeft = new CANSparkMax(DTL_IDs[0], MotorType.kBrushless);
    BackLeft = new CANSparkMax(DTL_IDs[1], MotorType.kBrushless);
    FrontRight = new CANSparkMax(DTR_IDs[0], MotorType.kBrushless);
    BackRight = new CANSparkMax(DTR_IDs[1], MotorType.kBrushless);

    BackLeft.setInverted(true);
    BackRight.setInverted(true);
    FrontRight.setInverted(true);
    FrontLeft.setInverted(true);

    FrontRightEnc = FrontRight.getEncoder();
    FrontLeftEnc = FrontLeft.getEncoder();
    BackRightEnc = BackRight.getEncoder();
    BackLeftEnc = BackLeft.getEncoder();

   // FrontLeftEnc.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    BackLeft.setSmartCurrentLimit(40);
    BackRight.setSmartCurrentLimit(40);
    FrontLeft.setSmartCurrentLimit(40);
    FrontRight.setSmartCurrentLimit(40);
    //BackLeft.setInverted(true);
    //BackRight.setInverted(true);

    FrontRight.setOpenLoopRampRate(0.5);
    FrontLeft.setOpenLoopRampRate(0.5);
    BackRight.setOpenLoopRampRate(0.5);
    BackLeft.setOpenLoopRampRate(0.5);


    LeftDrive = new SpeedControllerGroup(FrontLeft, BackLeft);
    RightDrive = new SpeedControllerGroup(FrontRight, BackRight);

    robot = new DifferentialDrive(LeftDrive, RightDrive);

    navx = new AHRS(Constants.NAVX_PORT);

    xbox = new XboxController(0);

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public static DriveTrain getInstance(){
		if(driveTrain == null) driveTrain = new DriveTrain();
    	return driveTrain;
    }

  public void Forward(XboxController joy){
    robot.arcadeDrive(joy.getY(Hand.kLeft), 0);
  }

  public void ControllerSplitArcade(XboxController joy){

    y = joy.getY(Hand.kLeft);
    x = joy.getX(Hand.kRight);

   robot.arcadeDrive(y*0.5 , -x*0.5);
  }

  public void SingleJoystickTiltArcade(Joystick joy){
    robot.arcadeDrive(joy.getY() * 0.5, joy.getX() * 0.3);
  }

  public void ControllerTankDrive(XboxController joy){
    robot.tankDrive(joy.getY(Hand.kLeft), joy.getY(Hand.kRight));
  }

  public void ControllerSingleArcade(XboxController joy){
    robot.arcadeDrive(joy.getY(Hand.kLeft), joy.getX(Hand.kLeft));
  }

  public void LimelightTracking(double leftPower, double rightPower){
    robot.tankDrive(leftPower, rightPower);
  }

  public void arcadeDriveAuto(double forward, double turn){
    robot.arcadeDrive(forward, turn);
  }

  public void DriveSetDistance(double powerForward, double turnPower, int distance){
    if(xbox.getBButton()){
    if(getOverAllAveragePosition() < distance){
    robot.curvatureDrive(powerForward, turnPower, false);
    System.out.print("run");
    }else{
      robot.stopMotor();
    }
   }
  }

  public void DriveSetDistancePID(int distance, double maxPower){
    error = (distance - getOverAllAveragePosition());

    if(Math.abs(error) < 0.7) {
      integral += (error*.02);
    } else {
      integral = 0;
    }

    power = (Kp*error + Ki*integral) + 0.01;

    if(power > maxPower) {
      power = maxPower;
    }

    if(Math.abs(error) <= 0.2){
      gotToSetDistance = true;
    }else{
      gotToSetDistance = false;
    }

    robot.curvatureDrive(-power, 0, false);
  }

  public void TurnToAnglePID(double angle, double maxPowera){
    errora = (angle - navx.getYaw());

    if(Math.abs(errora) < 2) {
      integrala += (errora*.02);
    } else {
      integrala = 0;
    }

    powera = (Kpa*errora + Kia*integrala);

    if(powera > maxPowera) {
      powera = maxPowera;
    }

    if(Math.abs(errora) <= 0.5){
      gotToSetAngle = true;
    }else{
      gotToSetAngle = false;
    }

    robot.tankDrive(-powera, powera);
  }

  public void stop(){
    robot.stopMotor();
  }

  public void restEncoders(){
    if(xbox.getAButton()){
    FrontLeftEnc.setPosition(0);
    BackLeftEnc.setPosition(0);
    FrontRightEnc.setPosition(0);
    BackRightEnc.setPosition(0);
    }
  }

  public void restAngle(){
    if(xbox.getYButton()){
      navx.reset();
    }
  }

  public double getFrontRightRPM(){
    return FrontRightEnc.getVelocity();
  }

  public double getFrontRight(){
    return FrontRightEnc.getVelocity();
  }

  public double getFrontLeftRPM(){
    return FrontLeftEnc.getVelocity();
  }

  public double getBackRightRPM(){
    return BackRightEnc.getVelocity();
  }

  public double getBackLeftRPM(){
    return BackLeftEnc.getVelocity();
  }

  public double getLeftAveragePostition(){
    return -((FrontLeftEnc.getPosition() + BackLeftEnc.getPosition()) / 2);
  }

  public double getRightAveragePostition(){
    return ((FrontRightEnc.getPosition() + BackRightEnc.getPosition()) / 2);
  }

  public double getOverAllAveragePosition(){
    return (getLeftAveragePostition() + getRightAveragePostition()) / 2;
  }

  public double getHeading() {
    return 1;//navx.getYaw();//Math.IEEEremainder(navx.getAngle(), 360);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // public double getRate(){
  //   return BackLeftEnc.getCountsPerRevolution();
  // }

  // public double getDistance(){
  //   return BackLeftEnc.getDistancePerPulse
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(Rotation2d.fromDegrees(getHeading()), FrontLeftEnc.getPosition(),FrontLeftEnc.getPosition());

    SmartDashboard.putNumber("FrontRightEnc", getFrontRightRPM());
    SmartDashboard.putNumber("FrontLeftEnc", getFrontLeftRPM());
    SmartDashboard.putNumber("BackRightEnc", getBackRightRPM());
    SmartDashboard.putNumber("BackLeftEnc", getBackLeftRPM());
    SmartDashboard.putNumber("OverallAverage", getOverAllAveragePosition());
    SmartDashboard.putNumber("LeftEnc", getLeftAveragePostition());
    SmartDashboard.putNumber("RightEnc", getRightAveragePostition());
    SmartDashboard.putNumber("Y", y);
    SmartDashboard.putNumber("Power", power);
    SmartDashboard.putNumber("AnglePower", powera);
    SmartDashboard.putNumber("Integral", integral);
    SmartDashboard.putNumber("AngleIntegral", integrala);
    SmartDashboard.putBoolean("AtSetPosition", gotToSetDistance);
    SmartDashboard.putBoolean("AtSetAngle", gotToSetAngle);
    SmartDashboard.putNumber("Yaw", navx.getYaw());
    //sSmartDashboard.putNumber("Yaw", navx.getYaw());

    restEncoders();
    restAngle();
    //DriveSetDistance(0.2, 0, 5);

  }
}
