package frc.robot;

public sealed interface FieldTarget
    permits FieldTargets.FieldZone,
        FieldTargets.AlliancePoseTarget2d,
        FieldTargets.AlliancePoseTarget3d,
        FieldTargets.AllianceZoneTarget {
  String name();
}
