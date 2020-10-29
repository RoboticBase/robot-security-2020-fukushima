import { GeoBlock } from "./geo_block.ts";
import { Point } from "./point.ts";


export class GeoFence {
  geoBlocks = new Array<GeoBlock>();
  constructor(geoBlocksFilePath: string) {
    const loadedJson: Array<Array<Point>> = JSON.parse(Deno.readTextFileSync(geoBlocksFilePath));
    loadedJson.forEach((geoData) => {
      this.geoBlocks.push(new GeoBlock([geoData[0], geoData[1]]));
    });
  }
  isRange(targetPoint: Point) {
    return this.geoBlocks.some((geoBlock) => {
      return geoBlock.isRange(targetPoint);
    });
  }
}
