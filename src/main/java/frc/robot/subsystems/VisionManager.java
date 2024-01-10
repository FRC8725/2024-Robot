// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManager extends SubsystemBase {
  private static final VisionManager instance = new VisionManager();

  public static VisionManager getInstance() {
    return instance;
  }

  DoubleSubscriber tidSub = NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tid").subscribe(-1);
  DoubleArraySubscriber targetpose_robotspaceSub = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);

  public VisionManager() { }

  @Override
  public void periodic() { }

  public Transform3d getAprilTagRelative() {
    var bestCameraToTargetArray = targetpose_robotspaceSub.get();
    Transform3d bestCameraToTarget = new Transform3d();
    if (hasTarget()) {
      bestCameraToTarget = new Transform3d(
        new Translation3d (bestCameraToTargetArray[0], -bestCameraToTargetArray[1], bestCameraToTargetArray[2]),
        new Rotation3d(bestCameraToTargetArray[3], bestCameraToTargetArray[4], bestCameraToTargetArray[5])
      );
    }
    return bestCameraToTarget;
  }

  public boolean hasTarget() {
    return tidSub.get() != -1.;
  }

  boolean isFirstConnected = true;

  // public static double getdistanceToGoalInches(){
  //   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //   NetworkTableEntry ty = table.getEntry("ty");
  //   double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
  //   double limelightMountAngleDegrees = Constants.VisionConstants.klimelightMountAngleDegrees;
  //   double limelightLensHeightInches = Constants.VisionConstants.klimelightLensHeightInches;
  //   double goalHeightInches =  Constants.VisionConstants.kgoalHeightInches;

  //   double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  //   double angleToGoalRadians = Units.degreesToRadians(angleToGoalDegrees);

  //   //calculate distance
  //   double distanceToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
  //   return distanceToGoalInches;
  // }

  // public static double getDistanceToGoalHorizontalInches(double distanceToGoalInches) {
  //   if(distanceToGoalInches == -1) {
  //     distanceToGoalInches = getdistanceToGoalInches();
  //   }
  //   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //   NetworkTableEntry tx = table.getEntry("tx");
  //   double targetOffsetAngle_Horizontal = tx.getDouble(0.0);
  //   double targetOffsetRadians_Horizontal = Units.degreesToRadians(targetOffsetAngle_Horizontal);
  //   double distanceToGoalHorizontalInches = (Math.tan(targetOffsetRadians_Horizontal) * distanceToGoalInches) - Constants.VisionConstants.klimelightHorizontalOffsetInches;
  //   return distanceToGoalHorizontalInches;
  // }

  // public static double getApriltagId(){
  //   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //   NetworkTableEntry tid = table.getEntry("tid");
  //   double apriltag_id = tid.getDouble(0.0);
  //   return apriltag_id;
  // }
}
