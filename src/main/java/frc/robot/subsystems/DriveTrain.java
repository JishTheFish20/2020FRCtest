/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

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

   private double P,I,D, error, angle, setangle,integral;

   //public AHRS navx;

   private DifferentialDrive robot;

   private DifferentialDriveOdometry odometry;
   
  public DriveTrain() {
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

  // navx = new AHRS(Constants.NAVX_PORT);

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

    // if(Math.abs(x) < 0.1){
    //   x = 0;
    // }
    // if(Math.abs(y) < 0.1){
    //   y = 0;
    // }

    // x *= x;
    // y *= y;

    // if(joy.getX(Hand.kRight) < 0){
    //   x *= -1;
    // }
    // if(joy.getY(Hand.kLeft) < 0){
    //   y *= -1;
    // }
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
    if(getOverAllAveragePosition() <= distance){
    robot.arcadeDrive(powerForward, turnPower);
    }else{
      robot.stopMotor();
    }


  }

  public void turnTo(double setangle){
    //setangle = this.setangle;
		P = SmartDashboard.getNumber("P", 0.01);
		I = SmartDashboard.getNumber("I", 0.1);
		D = SmartDashboard.getNumber("D", 0);

		SmartDashboard.putNumber("SetAngle", setangle);

		double angle = navx.getYaw();

		error = (setangle - angle);

		integral += error*0.02;

	    derivative = (error - this.prevError) / .02;
		
		speed = error*P + integral*I + derivative*D;

	    robot.tankDrive(speed, -speed);
		prevError = error;

  }

  // public void tankDrive(XboxController joy){
  //   robot.arcadeDrive(joy.getY(Hand.kLeft), joy.getX(Hand.kRight));
  // }

  // public void tankDrive(XboxController joy){
  //   robot.arcadeDrive(joy.getY(Hand.kLeft), joy.getX(Hand.kRight));
  // }

  public void PathWeaverPath(){
    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    // }
  }

  // public void followPath(){
  //   // Trajectory.State goal = trajectory.sample(3.4); // sample the trajectory at 3.4 seconds from the beginning
  //   // ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);
  // }

  public void stop(){
    robot.stopMotor();
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
    return ((FrontLeftEnc.getPosition() + BackLeftEnc.getPosition()) / 2);
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
    SmartDashboard.putNumber("Y", y);
    //sSmartDashboard.putNumber("Yaw", navx.getYaw());

  }
}
