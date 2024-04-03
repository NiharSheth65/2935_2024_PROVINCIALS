// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionDriveToTargetCommand extends Command {
  
  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController driveVisionPID; 

  private int setPipelineNumber; 

  private double driveSetPoint; 

  private double dSlew = 100;

  private SlewRateLimiter drive_Limiter = new SlewRateLimiter(dSlew); 

  private double initTime; 

  private double driveMeasuredPoint;
  private double driveSpeed;  

  private boolean endCommand; 
 

  /** Creates a new VisionDriveToTargetCommand. */
  public VisionDriveToTargetCommand(DriveSubsystem drive, VisionSubsystem vision, int pipelineNumber, boolean end) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.VISION_SUBSYSTEM = vision; 
    this.DRIVE_SUBSYSTEM = drive; 

    this.driveVisionPID = new PIDController(VisionConstants.visionDriveKp, VisionConstants.visionDriveKi, VisionConstants.visionDriveKd); 

    this.endCommand = end;  
    this.setPipelineNumber = pipelineNumber; 

    addRequirements(VISION_SUBSYSTEM);
    addRequirements(DRIVE_SUBSYSTEM);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveVisionPID.reset();
    
    VISION_SUBSYSTEM.setPipeline(setPipelineNumber);
   
    driveSetPoint = 0; 
    initTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(VISION_SUBSYSTEM.limelightTargetSeen()){
      driveMeasuredPoint = VISION_SUBSYSTEM.getTy(); 
    } 
    else{
      // driveMeasuredPoint = driveSetPoint; 
    }

    driveSpeed = -driveVisionPID.calculate(driveMeasuredPoint, VisionConstants.limelightReadToIntakePitch); 

    if(driveSpeed > VisionConstants.limelightDriveSpeedLimit){
      driveSpeed = VisionConstants.limelightDriveSpeedLimit; 
    }

    else if(driveSpeed < -VisionConstants.limelightDriveSpeedLimit){
      driveSpeed = -VisionConstants.limelightDriveSpeedLimit; 
    }

    DRIVE_SUBSYSTEM.setTank(drive_Limiter.calculate(driveSpeed), drive_Limiter.calculate(driveSpeed));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endCommand == true){
      return true; 
    }

    else if(VISION_SUBSYSTEM.getTy() < VisionConstants.limelightReadToIntakePitch){
      return true; 
    }
    
    else if(Math.abs(System.currentTimeMillis() - initTime) > VisionConstants.limelightCommandTimeout){
      return true; 
    }

    else{
      return false; 
    }
  }
}
