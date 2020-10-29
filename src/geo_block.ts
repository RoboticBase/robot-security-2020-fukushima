import type { Point } from "./point.ts";

// Points for connecting diagonal lines
//
// Point1 *-----------------
//        |                |
//        |    GEOBlock    |
//        |                |
//        -----------------* Point2

type BlockPoints = [Point, Point];

export class GeoBlock {
  private hightLatitude: number;
  private rowLatitude: number;
  private hightLongitude: number;
  private rowLongitude: number;
  constructor(private points: BlockPoints) {
    if (this.points[0].latitude < this.points[1].latitude) {
      this.hightLatitude = this.points[1].latitude;
      this.rowLatitude = this.points[0].latitude;
    } else {
      this.hightLatitude = this.points[0].latitude;
      this.rowLatitude = this.points[1].latitude;
    }
    if (this.points[0].longitude < this.points[1].longitude) {
      this.hightLongitude = this.points[1].longitude;
      this.rowLongitude = this.points[0].longitude;
    } else {
      this.hightLongitude = this.points[0].longitude;
      this.rowLongitude = this.points[1].longitude;
    }
  }
  isRange(targetPoint: Point) {
    const isLatitudeRange = this.hightLatitude >= targetPoint.latitude && this.rowLatitude <= targetPoint.latitude;
    const isLongitudeRange = this.hightLongitude >= targetPoint.longitude && this.rowLongitude <= targetPoint.longitude;
    return isLatitudeRange && isLongitudeRange;
  }
}
