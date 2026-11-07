// Provided by Rocket Robotics, FTC 21615
// Copyright (c) 2026 Maxim Vanier
// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Reads the AprilTag Cluster center computed by apriltag_cluster.py running
 * as a Python (SnapScript) pipeline on the Limelight 3A.
 *
 * During init, gamepad 1: B = red alliance, X = blue alliance, A = any (no filter).
 */
@TeleOp(name = "Cluster Center", group = "Vision")
public class ClusterCenterTeleOp extends LinearOpMode {

    // Index of the Python pipeline in the Limelight web UI.
    private static final int CLUSTER_PIPELINE = 1;

    // Camera pitch in degrees, + = tilted up (0 = level, 90 = straight up).
    // Measure this on the robot.
    private static final double CAMERA_PITCH_DEG = 30.0;

    // Python input [1] values
    private static final double ANY = 0, BLUE = 1, RED = 2;

    // llpython layout (must match apriltag_cluster.py). Distances are in inches,
    // angles in degrees: use Math.toRadians() before Math.sin/cos/tan.
    private static final int CLUSTER_ID = 0, NUM_TAGS = 1, RANGE = 2, BEARING = 3,
            HEIGHT = 4, ELEVATION = 5, STICKER_PITCH = 6, ROLL = 7;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(CLUSTER_PIPELINE);
        limelight.start();

        double alliance = ANY;
        while (opModeInInit()) {
            if (gamepad1.b) alliance = RED;
            if (gamepad1.x) alliance = BLUE;
            if (gamepad1.a) alliance = ANY;
            telemetry.addData("Alliance (B red / X blue / A any)",
                    alliance == RED ? "RED" : alliance == BLUE ? "BLUE" : "ANY");
            telemetry.update();
        }
        // Inputs: [0] 1 = inputs valid (0 makes the script use its own defaults),
        // [1] alliance, [2] camera pitch in degrees
        limelight.updatePythonInputs(new double[] {1, alliance, CAMERA_PITCH_DEG, 0, 0, 0, 0, 0});

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double[] py = (result != null) ? result.getPythonOutput() : null;

            if (py != null && py.length >= 8 && py[CLUSTER_ID] >= 0) {
                telemetry.addData("Cluster", "%d  (%d/4 tags)", (int) py[CLUSTER_ID], (int) py[NUM_TAGS]);
                telemetry.addData("Range (in)", "%.1f", py[RANGE]);
                telemetry.addData("Bearing (+ left)", "%.2f°", py[BEARING]);
                telemetry.addData("Height (in)", "%.1f", py[HEIGHT]);
                telemetry.addData("Elevation", "%.2f°", py[ELEVATION]);
                telemetry.addData("Sticker pitch", "%.1f°", py[STICKER_PITCH]);
                telemetry.addData("Roll", "%.1f°  %s", py[ROLL],
                        Math.abs(py[ROLL]) < 90 ? "right-side up" : "upside down");
            } else {
                telemetry.addLine("No cluster");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
