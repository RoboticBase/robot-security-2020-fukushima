import { DEFAULT_ALLOWABLE_COUNT, DEFAULT_RANGE_METERS } from "./const.ts";
import { Point } from "./types.ts";

class StuckChecker {
  constructor(
    private allowableCount: number = DEFAULT_ALLOWABLE_COUNT,
    private range: number = DEFAULT_RANGE_METERS,
  ) {
  }

  // Calculate the earth as a true sphere.
  // Correctly, it should be calculated considering GRS80
  calculateDistance(point1: Point, point2: Point) {
    const EARTH_DISTANCE = 6371000;
    const RADIUS = 180;

    const distanceMeters = EARTH_DISTANCE * Math.acos(
      Math.cos(point1.latitude / RADIUS * Math.PI) *
          Math.cos((point2.longitude - point1.longitude) / RADIUS * Math.PI) *
          Math.cos(point2.latitude / RADIUS * Math.PI) +
        Math.sin(point1.latitude / RADIUS * Math.PI) *
          Math.sin(point2.latitude / RADIUS * Math.PI),
    );
    return distanceMeters;
  }
}
