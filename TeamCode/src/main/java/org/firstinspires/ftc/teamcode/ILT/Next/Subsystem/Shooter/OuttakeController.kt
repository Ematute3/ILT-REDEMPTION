package org.firstinspires.ftc.teamcode.robot.subsystems.shooter

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot.AimbotTable
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.data.enums.OuttakeMode
import org.firstinspires.ftc.teamcode.robot.data.enums.TurretMode

/**
 * OuttakeController - Coordinates all shooter subsystems.
 * Updates Hood and Flywheel based on distance data from Limelight or Odometry.
 */
object OuttakeController : Subsystem {

    // ==================== MANUAL AIM SETTINGS ====================
    var manualAimDistance = 48  // inches (for MANUAL mode)
        private set

    private var prevOuttakeMode: OuttakeMode? = null
    private var prevTurretMode: TurretMode? = null

    override fun periodic() {
        val outtakeModeChanged = RobotState.outtakeMode != prevOuttakeMode
        val turretModeChanged = RobotState.turretMode != prevTurretMode

        // ========================================
        // HANDLE OUTTAKE MODES (Hood + Flywheel)
        // ========================================
        when (RobotState.outtakeMode) {
            OuttakeMode.IDLE -> {
                if (outtakeModeChanged) {
                    RobotState.hoodPosition = 0.0
                    FlyWheel.stop.schedule()
                }
            }

            OuttakeMode.MANUAL -> {
                //applyManualAim()
            }

            OuttakeMode.AUTO_ODOMETRY -> {
                if (RobotState.poseValid) {
                   // applyAimFromDistance(RobotState.distanceToGoalOdometry, "Odometry")
                }
            }

            OuttakeMode.AUTO_LIMELIGHT -> {
               // applyLimelightAim()
            }
        }

        // ========================================
        // HANDLE TURRET MODES (Bridge to Turret Subsystem)
        // ========================================
        when (RobotState.turretMode) {
            TurretMode.IDLE -> Turret.currentState = Turret.State.IDLE

            TurretMode.MANUAL -> Turret.currentState = Turret.State.MANUAL

            TurretMode.LIMELIGHT -> Turret.currentState = Turret.State.LIMELIGHT

            TurretMode.ODOMETRY -> {
                // If we see a target, use the Fused logic, otherwise stay on Odo
                if (RobotState.limelightHasTarget) {
                    Turret.currentState = Turret.State.FUSED
                } else {
                    Turret.currentState = Turret.State.ODOMETRY
                }
            }
        }

        prevOuttakeMode = RobotState.outtakeMode
        prevTurretMode = RobotState.turretMode

        // Telemetry for debugging distance logic
        ActiveOpMode.telemetry.run {
            addData("--- Outtake ---", "")
            addData("Mode", RobotState.outtakeMode)
            val dist = if (RobotState.outtakeMode == OuttakeMode.AUTO_LIMELIGHT)
                RobotState.distanceToGoalLimelight else RobotState.distanceToGoalOdometry
            addData("Active Dist", "%.1f".format(dist ?: 0.0))
        }
    }

    // ==================== AIM LOGIC ====================

  /*  private fun applyManualAim() {
        val snapped = AimbotTable.snapToValidDistance(manualAimDistance)
        // Fixed: AimbotTable.getValues now returns non-null Pair
        val values = AimbotTable.getValues(snapped.toDouble())

        RobotState.hoodPosition = values.first
        FlyWheel.setTargetVelocity(values.second)
    }

   */

  /*  private fun applyAimFromDistance(distance: Double, source: String) {
        // Fixed: Uses the clamped version of getValues to prevent NullPointer/Deque errors
        val values = AimbotTable.getValues(distance)

        // Apply with competition offsets
        RobotState.hoodPosition = (values.first + 0.06).coerceIn(0.0, 1.0)
        FlyWheel.setTargetVelocity(values.second + 100.0)

        ActiveOpMode.telemetry.addData("Aim Source", source)
    }

    private fun applyLimelightAim() {
        val llDistance = RobotState.distanceToGoalLimelight

        when {
            llDistance != null && RobotState.limelightHasTarget -> {
                applyAimFromDistance(llDistance, "Limelight")
            }
            RobotState.poseValid -> {
                applyAimFromDistance(RobotState.distanceToGoalOdometry, "Odo Fallback")
            }
            else -> {
                // Last resort: Use a default middle-field distance
                applyAimFromDistance(72.0, "DEFAULT")
            }
        }
    }

    fun canShoot(): Boolean {
        // Only allow shooting if mechanical systems are locked on
        return RobotState.flywheelAtSpeed && RobotState.turretAligned
    }

    // ==================== MODE COMMANDS ====================

    val idleMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.IDLE
        RobotState.turretMode = TurretMode.IDLE
    }

    val manualMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.MANUAL
        RobotState.turretMode = TurretMode.MANUAL
    }

    val autoOdometryMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.AUTO_ODOMETRY
        RobotState.turretMode = TurretMode.ODOMETRY
    }

    val autoLimelightMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.AUTO_LIMELIGHT
        RobotState.turretMode = TurretMode.LIMELIGHT
    }

   */
}