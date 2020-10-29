import { assertEquals } from "https://deno.land/std@0.73.0/testing/asserts.ts";
import { GeoBlock } from "../src/geo_block.ts";
import { GeoFence } from "../src/geo_fence.ts";
import { Point } from "../src/point.ts";
import { Parameters } from "./utils/parameters_test.ts";

const geoBlocksFilePath = "./tests/data/geo_blocks.json";
const geoFence = new GeoFence(geoBlocksFilePath);

Deno.test("Test geo blocks length", () => {
  assertEquals(1, geoFence.geoBlocks.length);
});

Parameters.test<Point>(
  "Test geo fence when isRange is true.",
  [
    { latitude: 37.49213822, longitude: 139.93015002 },
    { latitude: 37.49253810, longitude: 139.93071201 },
    { latitude: 37.49263122, longitude: 139.93081622 },
  ],
  (point: Point) => {
    assertEquals(true, geoFence.isRange(point));
  },
);

