/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveSetDistance;
import frc.robot.commands.LimeLightTracking;
import frc.robot.commands.MoveShooterIntake;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.Tracking;
import frc.robot.commands.TurnToAngle;
//import frc.robot.commands.StopDrive;
import frc.robot.commands.XboxControllerSplitArcade;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.WheelOfFourtune;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //XboxController Driver = new XboxController(Constants.DRIVER);
  XboxController Driver = new XboxController(Constants.DRIVER);

  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
   private final LimeLight limeLight = new LimeLight();
  // private final ShooterIntake shooterIntake = new ShooterIntake(); 
  // private final WheelOfFourtune wof = new WheelOfFourtune();
  // private final LEDS Led = new LEDS();
 
 // private final XboxControllerSplitArcade xboxControllerSplitArcade = new Mecanum(Driver);
  private final XboxControllerSplitArcade xboxControllerSplitArcade = new XboxControllerSplitArcade(driveTrain,Driver);
  //private final StopDrive stopDrive = new StopDrive();
  //private final SpinShooter spinShooter = new SpinShooter();
   //private final LimeLightTracking limeLightTracking = new LimeLightTracking(Driver);




  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(new XboxControllerSplitArcade(driveTrain, Driver));
    //shooter.setDefaultCommand(new SpinShooter(shooter, Driver));
    //limeLight.setDefaultCommand(new LimeLightTracking(limeLight, driveTrain, Driver));
    //shooterIntake.setDefaultCommand(new MoveShooterIntake(shooterIntake, Driver));
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(Driver, 2).whenHeld(new DriveSetDistance(driveTrain, 0.2, 10));
    new JoystickButton(Driver, 3).whenHeld(new TurnToAngle(driveTrain, 0.25, 130));

    //new JoystickButton(Driver, 1).whenHeld(limeLightTracking);
    //new JoystickButton(Driver, Driver.getAButton()).
   // new JoystickButton(Driver, 4).whenHeld(xboxControllerSplitArcade);
    //new JoystickButton(Driver, 3).whenHeld(new LimeLightTracking(limeLight, driveTrain, Driver));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new  LimeLightTracking(limeLight, driveTrain, Driver);
  }
}
