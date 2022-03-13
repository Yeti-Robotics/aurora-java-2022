// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
    public static PhotonCamera camera = new PhotonCamera("yetiworm");

    public static double getX(){
        PhotonTrackedTarget latestTarget = camera.getLatestResult().getBestTarget();
        return (latestTarget == null) ? 0.0 : latestTarget.getYaw();
    }
}
