// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.LimelightConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
    private static PhotonCamera camera = new PhotonCamera("yetiworm");

    public static double getX() {
        PhotonTrackedTarget latestTarget = camera.getLatestResult().getBestTarget();
        return (latestTarget == null) ? 0.0 : latestTarget.getYaw();
    }

    public static double getY() {
        PhotonTrackedTarget latestTarget = camera.getLatestResult().getBestTarget();
        return (latestTarget == null) ? 0.0 : latestTarget.getPitch();
    }

    public static double getDistance() {
        if (!camera.getLatestResult().hasTargets()) return 0.0;

        double angleToGoalDegrees = LimelightConstants.MOUNTING_ANGLE + getY();
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches =
                (LimelightConstants.GOAL_HEIGHT - LimelightConstants.LIMELIGHT_HEIGHT)
                        / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }
}
