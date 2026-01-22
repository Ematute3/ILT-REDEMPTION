package org.firstinspires.ftc.teamcode.robot.subsystems.drive

import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState


/**
 * DriveTrain subsystem.
 * Handles mecanum drive via Pedro, pose tracking, and zone checking.
 */
object DriveTrain : Subsystem {

    // IMU for heading (backup/reset)
    private lateinit var imu: IMUEx

    // Zone checker
    private lateinit var zoneChecker: ZoneChecker

    // Initialization flag
    private var isInitialized = false

    override fun initialize() {
        try {
            imu = IMUEx(RobotConfig.Hardware.IMU, Direction.RIGHT, Direction.UP)
            zoneChecker = ZoneChecker()
            isInitialized = true
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("DriveTrain Error", e.message)
            isInitialized = false
        }
    }

    // Pedro handles driving via its default command
    override val defaultCommand: Command
        get() = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
            false  // field centric
        )

    override fun periodic() {
        if (!isInitialized) return

        // Update pose from Pedro follower
        try {
            PedroComponent.follower?.let { follower ->
                RobotState.updatePose(
                    follower.pose.x,
                    follower.pose.y,
                    follower.heading
                )
            } ?: run {
                RobotState.invalidatePose()
            }
        } catch (e: Exception) {
            RobotState.invalidatePose()
            ActiveOpMode.telemetry.addData("Pose Error", e.message)
        }

        // Update zone status
        if (RobotState.poseValid) {
            RobotState.inShootZone = zoneChecker.inShootZone(
                RobotState.currentX,
                RobotState.currentY
            )

            // Calculate distance to goal
            val dx = RobotState.goalX - RobotState.currentX
            val dy = RobotState.goalY - RobotState.currentY
            RobotState.distanceToGoalOdometry = kotlin.math.sqrt(dx * dx + dy * dy)
        }

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("Pose Valid", RobotState.poseValid)
            addData("Position", "(%.1f, %.1f)".format(RobotState.currentX, RobotState.currentY))
            addData("Heading", "%.1f°".format(Math.toDegrees(RobotState.currentHeading)))
            addData("In Shoot Zone", RobotState.inShootZone)
            addData("Distance to Goal", "%.1f".format(RobotState.distanceToGoalOdometry))
        }
    }

    /**
     * Reset the IMU heading.
     */
    fun resetImu() {
        if (isInitialized) {
            imu.zero()
        }
    }

    /**
     * Set the alliance (affects goal position and zone checking).
     */
    fun setAlliance(alliance: Alliance) {
        RobotConfig.alliance = alliance
    }

    /**
     * Get the IMU heading (for backup/verification).
     */

}