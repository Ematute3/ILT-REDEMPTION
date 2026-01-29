package org.firstinspires.ftc.teamcode.robot.data.config

import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.IntakeState
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Motif
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.OuttakeMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.ShootMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.TurretMode


/**
 * Centralized robot state.
 * Single source of truth for all shared state variables.
 * Subsystems read from and write to this object.
 */
@Configurable
object RobotState {

    // ==================== POSE STATE ====================
    var currentX: Double = 0.0
    var currentY: Double = 0.0
    var currentHeading: Double = 0.0
    var poseValid: Boolean = false

    fun updatePose(x: Double, y: Double, heading: Double) {
        currentX = x
        currentY = y
        currentHeading = heading
        poseValid = true
    }

    fun invalidatePose() {
        poseValid = false
    }

    // ==================== GOAL STATE ====================
    val goalX: Double
        get() = if (RobotConfig.alliance == Alliance.RED) {
            RobotConfig.RED_GOAL_X
        } else {
            RobotConfig.BLUE_GOAL_X
        }

    val goalY: Double = RobotConfig.GOAL_Y

    // ==================== MODE STATE ====================
    var outtakeMode: OuttakeMode = OuttakeMode.IDLE
    var turretMode: TurretMode = TurretMode.IDLE
    var shootMode: ShootMode = ShootMode.IDLE

    // ==================== SHOOTER STATE ====================
    var flywheelOn: Boolean = false
    var flywheelAtSpeed: Boolean = false
    var flywheelVelocity: Double = 0.0
    var targetFlywheelVelocity: Double = 0.0

    var turretYaw: Double = 0.0       // Current turret angle from motor encoder
    var turretAligned: Boolean = false // True when turret is aimed at target
    var turretHomed: Boolean = false   // True after homing sequence completes

    var hoodPosition: Double = 0.0

    // ==================== VISION STATE ====================
    var limelightReady: Boolean = false
    var limelightHasTarget: Boolean = false
    var limelightTx: Double = 0.0
    var limelightTy: Double = 0.0
    var limelightTa: Double = 0.0
    var detectedMotif: Motif = Motif.NONE

    var distanceToGoalOdometry: Double = 0.0
    var distanceToGoalLimelight: Double? = null

    // ==================== INTAKE STATE ====================
    var intakeState: IntakeState = IntakeState.STOPPED

    // ==================== ZONE STATE ====================
    var inShootZone: Boolean = false

    // ==================== HELPER FUNCTIONS ====================

    /**
     * Check if robot is ready to shoot
     */
    fun canShoot(): Boolean {
        return poseValid &&
                inShootZone &&
                flywheelAtSpeed &&
                turretAligned
    }

    /**
     * Check if auto-aim modes can be used
     */
    fun canUseAutoModes(): Boolean = poseValid

    /**
     * Check if Limelight modes can be used
     */
    fun canUseLimelightModes(): Boolean = limelightReady && limelightHasTarget

    /**
     * Reset all state to defaults
     */
    fun reset() {
        poseValid = false
        outtakeMode = OuttakeMode.IDLE
        turretMode = TurretMode.IDLE
        shootMode = ShootMode.IDLE
        flywheelOn = false
        flywheelAtSpeed = false
        limelightReady = false
        limelightHasTarget = false
        intakeState = IntakeState.STOPPED
        inShootZone = false
    }
}