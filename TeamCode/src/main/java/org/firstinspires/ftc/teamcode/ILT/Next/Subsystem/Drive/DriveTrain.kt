package org.firstinspires.ftc.teamcode.robot.subsystems.drive

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.sqrt


/**
 * DriveTrain subsystem.
 *
 * IMPORTANT: Pedro driving is handled by calling driverControlled() in periodic().
 * This is NOT done via defaultCommand - we must explicitly call it every loop!
 */
object DriveTrain : Subsystem {

    private var isInitialized = false

    // The Pedro driver controlled command - created once, called every loop


    // ==================== INITIALIZATION ====================
    override fun initialize() {
        try {
            // Create the driver controlled command
            // This uses Gamepads which wraps the gamepad inputs


            isInitialized = true
            ActiveOpMode.telemetry.addData("DriveTrain", "Initialized OK")
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("DriveTrain Error", e.message)
            isInitialized = false
        }
    }


    // ==================== PERIODIC - MUST CALL driverControlled() HERE ====================
    override fun periodic() {

        // THIS IS THE KEY LINE - actually run the Pedro driving!
        // The () invokes the command's update logic

        // Update pose from Pedro follower
        val follower = PedroComponent.follower

        if (follower != null) {
            try {
                val pose = follower.pose
                RobotState.currentX = pose.x
                RobotState.currentY = pose.y
                RobotState.currentHeading = pose.heading
                RobotState.poseValid = true

                // Calculate distance to goal
                val dx = RobotState.goalX - pose.x
                val dy = RobotState.goalY - pose.y
                RobotState.distanceToGoalOdometry = sqrt(dx * dx + dy * dy)
            } catch (e: Exception) {
                RobotState.poseValid = false
            }
        } else {
            RobotState.poseValid = false
        }

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("=== DRIVETRAIN ===", "")
            addData("Pose Valid", RobotState.poseValid)
            if (RobotState.poseValid) {
                addData("X", "%.1f".format(RobotState.currentX))
                addData("Y", "%.1f".format(RobotState.currentY))
                addData("Heading", "%.1f°".format(Math.toDegrees(RobotState.currentHeading)))
                addData("Dist to Goal", "%.1f\"".format(RobotState.distanceToGoalOdometry))
            }
        }
    }

    // ==================== HELPER FUNCTIONS ====================

    /*fun resetPose(x: Double, y: Double, heading: Double) {
        PedroComponent.follower?.pose = com.pedropathing.localization.Pose(x, y, heading)
    }

     */

    /**
     * Switch between robot centric and field centric driving.
     * Must recreate the command with new setting.
     */
   /* fun setRobotCentric(robotCentric: Boolean) {
        if (!isInitialized) return

        driverControlled = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
            robotCentric
        )
    }

    */
   val resetHeading = InstantCommand{

        follower.pose = Pose(RobotState.currentX, RobotState.currentY, 0.0)
    }
}