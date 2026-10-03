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
 * During init, gamepad 1: B = red alliance, X = blue alliance.
 */
@TeleOp(name = "Cluster Center", group = "Vision")
public class ClusterCenterTeleOp extends LinearOpMode {

    // Index of the Python pipeline in the Limelight web UI.
    private static final int CLUSTER_PIPELINE = 1;

    // Camera pitch in degrees, + = tilted up (0 = level, 90 = straight up).
    // Measure this on the robot.
    private static final double CAMERA_PITCH_DEG = 18.0;

    // llrobot[0] values
    private static final double RED = 1, BLUE = 0;

    // llpython layout (must match apriltag_cluster.py)
    private static final int CLUSTER_ID = 0, NUM_TAGS = 1, RANGE = 2, BEARING = 3,
            HEIGHT = 4, ELEVATION = 5, STICKER_PITCH = 6, ROLL = 7;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(CLUSTER_PIPELINE);
        limelight.start();

        double alliance = BLUE;
        while (opModeInInit()) {
            if (gamepad1.b) alliance = RED;
            if (gamepad1.x) alliance = BLUE;
            telemetry.addData("Alliance (B red / X blue)", alliance == BLUE ? "BLUE" : "RED");
            telemetry.update();
        }
        limelight.updatePythonInputs(new double[] {alliance, CAMERA_PITCH_DEG, 0, 0, 0, 0, 0, 0});

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
