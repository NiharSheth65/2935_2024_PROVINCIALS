// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionDriveAndAlignCommand extends Command {
  /** Creates a new VisionDriveAndAlignCommand. */

  private VisionSubsystem VISION_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 

  private PIDController driveVisionPID; 
  private PIDController turnVisionPID; 
  
  private int setPipelineNumber; 

  private double driveSetPoint; 
  private double turnSetPoint; 

  private double dSlew = 100;
  private double tSlew = 2; 
  
  private SlewRateLimiter drive_Limiter = new SlewRateLimiter(dSlew); 
  private SlewRateLimiter turn_Limiter = new SlewRateLimiter(tSlew); 

  private double initTime; 

  private double driveMeasuredPoint;
  private double turnMeasuredPoint; 

  private double driveSpeed;  
  private double turnSpeed;


  private boolean endCommand; 

  public VisionDriveAndAlignCommand(DriveSubsystem drive, VisionSubsystem vision, int pipeline, boolean end) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.VISION_SUBSYSTEM = vision; 

    this.turnVisionPID = new PIDController(VisionConstants.visionTurnKp, VisionConstants.visionTurnKi, VisionConstants.visionTurnKd); 
    this.driveVisionPID = new PIDController(VisionConstants.visionDriveKp, VisionConstants.visionDriveKi, VisionConstants.visionDriveKd); 
    
    this.endCommand = end; 
    this.setPipelineNumber = pipeline;

    addRequirements(DRIVE_SUBSYSTEM);
    addRequirements(VISION_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnVisionPID.reset();
    driveVisionPID.reset();
    VISION_SUBSYSTEM.setPipeline(setPipelineNumber);
    initTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(VISION_SUBSYSTEM.limelightTargetSeen()){
      driveMeasuredPoint = VISION_SUBSYSTEM.getTy(); 
      turnMeasuredPoint = VISION_SUBSYSTEM.getTx(); 
    } 
    else{
      // driveMeasuredPoint = driveSetPoint; 
    }

    driveSpeed = -driveVisionPID.calculate(driveMeasuredPoint, VisionConstants.limelightReadToIntakePitch); 
    turnSpeed = turnVisionPID.calculate(turnMeasuredPoint, 0); 

    if(driveSpeed > VisionConstants.limelightDriveSpeedLimit){
      driveSpeed = VisionConstants.limelightDriveSpeedLimit; 
    }

    else if(driveSpeed < -VisionConstants.limelightDriveSpeedLimit){
      driveSpeed = -VisionConstants.limelightDriveSpeedLimit; 
    }

    if(turnSpeed > VisionConstants.limelightTurnSpeedLimit){
      turnSpeed = VisionConstants.limelightTurnSpeedLimit; 
    }

    else if(turnSpeed < -VisionConstants.limelightTurnSpeedLimit){
      turnSpeed = -VisionConstants.limelightTurnSpeedLimit; 
    }

    DRIVE_SUBSYSTEM.setTank(drive_Limiter.calculate(driveSpeed) + turn_Limiter.calculate(turnSpeed), drive_Limiter.calculate(driveSpeed) - turn_Limiter.calculate(turnSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endCommand == true){
      return true; 
    }
    
    else if(Math.abs(VISION_SUBSYSTEM.getTx()) < VisionConstants.fineAlignmentTolerance && VISION_SUBSYSTEM.getTy() < VisionConstants.limelightReadToIntakePitch){
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
