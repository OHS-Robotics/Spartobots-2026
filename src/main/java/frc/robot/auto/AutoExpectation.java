package frc.robot.auto;

public enum AutoExpectation {
  PARK_FIRST("0-4"),
  PRELOAD_ONLY("4-8"),
  SINGLE_CYCLE("8-12"),
  DOUBLE_CYCLE("10-14");

  private final String pointsBand;

  AutoExpectation(String pointsBand) {
    this.pointsBand = pointsBand;
  }

  public String pointsBand() {
    return pointsBand;
  }
}
