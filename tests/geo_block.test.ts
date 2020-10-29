import { assertEquals } from "https://deno.land/std@0.73.0/testing/asserts.ts";
import { GeoBlock } from "../src/geo_block.ts";
import { Point } from "../src/point.ts";
import { Parameters } from "./utils/parameters_test.ts";

const geoBlock = new GeoBlock(
  [
    { latitude: 37.49213822, longitude: 139.93015002 },
    { latitude: 37.49263122, longitude: 139.93081622 },
  ],
);
Parameters.test<Point>(
  "Test geo block when isRange is true.",
  [
    { latitude: 37.49213822, longitude: 139.93015002 },
    { latitude: 37.49253810, longitude: 139.93071201 },
    { latitude: 37.49263122, longitude: 139.93081622 },
  ],
  (point: Point) => {
    assertEquals(true, geoBlock.isRange(point));
  },
);

Parameters.test<Point>(
  "Test geo block when isRange is false.",
  [
    { latitude: 37.49213822, longitude: 139.93015001 },
    { latitude: 37.49213821, longitude: 139.93015002 },
    { latitude: 37.49263122, longitude: 139.93081623 },
    { latitude: 37.49263123, longitude: 139.93081622 },
    { latitude: -37.49263120, longitude: 139.93081622 },
    { latitude: 37.49263120, longitude: -139.93081622 },
  ],
  (point: Point) => {
    assertEquals(false, geoBlock.isRange(point));
  },
);