// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardSetDistance extends Command {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private double distanceInInches;

  private PIDController leftWheelPowerPIDController; 
  private PIDController rightWheelPowerPIDController; 
  private PIDController gyroPIDController; 
  
  private double gryoTarget; 

  private boolean driveCompleted; 

  private SlewRateLimiter left_Limiter = new SlewRateLimiter(10); 
  private SlewRateLimiter right_Limiter = new SlewRateLimiter(10); 

  private double maxDriveSpeed; 

  private double initTime; 

  /** Creates a new DriveForwardSetDistance. */
  public DriveForwardSetDistance(DriveSubsystem drive, double distance, double maxSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.DRIVE_SUBSYSTEM = drive; 
    this.distanceInInches = distance;    

    this.leftWheelPowerPIDController = new PIDController(0.075, 0.01, 0); 
    this.rightWheelPowerPIDController = new PIDController(0.075, 0.01, 0); 
    this.gyroPIDController = new PIDController(0.005, 0, 0); 

    this.maxDriveSpeed = maxSpeed; 

    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DRIVE_SUBSYSTEM.resetEncoders();
    DRIVE_SUBSYSTEM.zeroHeading();
    
    leftWheelPowerPIDController.reset();
    rightWheelPowerPIDController.reset();
    gyroPIDController.reset();

    gryoTarget = DRIVE_SUBSYSTEM.getYaw(); 

    driveCompleted = false; 

    initTime = System.currentTimeMillis(); 

    System.out.println("DRIVE ACTIVATED!"); 

    VisionConstants.driveStartGyroAngle = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftWheelOutput = leftWheelPowerPIDController.calculate(DRIVE_SUBSYSTEM.getLeftEncoderToInches(), distanceInInches); 
    double rightWheelOutput = rightWheelPowerPIDController.calculate(DRIVE_SUBSYSTEM.getRightEncoderToInches(), distanceInInches); 
    double gyroSpeed = -gyroPIDController.calculate(DRIVE_SUBSYSTEM.getYaw(), gryoTarget); 

    if(leftWheelOutput > maxDriveSpeed){
      leftWheelOutput = maxDriveSpeed; 
    }

    else if(leftWheelOutput < -maxDriveSpeed){
      leftWheelOutput = -maxDriveSpeed; 
    }

    if(rightWheelOutput > maxDriveSpeed){
      rightWheelOutput = maxDriveSpeed;
    }

    else if(rightWheelOutput < -maxDriveSpeed){
      rightWheelOutput = -maxDriveSpeed;
    }

    DRIVE_SUBSYSTEM.setTank(left_Limiter.calculate(leftWheelOutput + gyroSpeed), right_Limiter.calculate(rightWheelOutput - gyroSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVE ENDED!"); 
    DRIVE_SUBSYSTEM.stop();
    VisionConstants.driveStartGyroAngle = DRIVE_SUBSYSTEM.getYaw();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    // if(Math.abs(distanceInInches - DRIVE_SUBSYSTEM.getAverageEncoderDistanceInInches()) < 2){
    //   return true; 
    // }

    if(Math.abs(DRIVE_SUBSYSTEM.getAverageEncoderDistanceInInches()) > Math.abs(distanceInInches)){
      return true; 
    }

    else if(Math.abs(System.currentTimeMillis() - initTime) > 2000){
      return true; 
    }

    else{
      return false; 
    }
  }
}
