import {
  DEFAULT_ALLOWABLE_COUNT,
  DEFAULT_ENTITY_ID,
  DEFAULT_FIWARE_SERVICE,
  DEFAULT_FIWARE_SERVICEPATH,
  DEFAULT_LOOP_SLEEP_SECOND,
  DEFAULT_RANGE_METERS,
  POINT_HISTORY_ID_PREFIX,
} from "./const.ts";
import { OrionClient } from "./orion_client.ts";
import {
  NGSIStuckCheckerEntity,
  NGSIPoseAttribute,
  Point,
  PointHistory,
  NGSIPointHistoryAttribute,
} from "./types.ts";
import { log } from "../deps.ts";
import { sleep } from "./utils.ts";

const logger = log.getLogger();

export class StuckChecker {
  private orionClient: OrionClient;
  constructor(
    private allowableCount: number = DEFAULT_ALLOWABLE_COUNT,
    private range: number = DEFAULT_RANGE_METERS,
  ) {
    this.orionClient = new OrionClient();
  }
  async startLoop(
    entityId: string = DEFAULT_ENTITY_ID,
    fiwareService: string = DEFAULT_FIWARE_SERVICE,
    fiwareServicePath: string = DEFAULT_FIWARE_SERVICEPATH,
  ) {
    while (true) {
      await this.execute(entityId, fiwareService, fiwareServicePath);
      await sleep(DEFAULT_LOOP_SLEEP_SECOND);
    }
  }
  async execute(
    entityId: string,
    fiwareService: string,
    fiwareServicePath: string,
  ) {
    const poseAttr = await this.orionClient.getAttrs<NGSIPoseAttribute>(
      entityId,
      "pose",
      fiwareService,
      fiwareServicePath,
    );
    const currentPoint: Point = {
      latitude: poseAttr.value.point.latitude,
      longitude: poseAttr.value.point.longitude,
    };
    if (
      await this.isStuck(
        currentPoint,
        entityId,
        fiwareService,
        fiwareServicePath,
      )
    ) {
      logger.error(
        { "message": `Error, "${entityId}" stuck`, point: currentPoint },
      );
    } else {
      logger.info({ "message": `OK, "${entityId}"`, point: currentPoint });
    }
    await this.updatePoseHistory(
      currentPoint,
      entityId,
      fiwareService,
      fiwareServicePath,
    );
  }
  async updatePoseHistory(
    currentPoint: Point,
    entityId: string,
    fiwareService: string,
    fiwareServicePath: string,
  ) {
    const entity = await this.orionClient.getEntity<NGSIStuckCheckerEntity>(
      `${POINT_HISTORY_ID_PREFIX}_${entityId}`,
      fiwareService,
      fiwareServicePath,
    );
    const pointHistory = this.getUpdatedPointHistory(
      entity.pointHistory.value.history,
      currentPoint,
    );
    await this.orionClient.patchAttr<NGSIPointHistoryAttribute>(
      `${POINT_HISTORY_ID_PREFIX}_${entityId}`,
      {
        pointHistory: {
          type: "object",
          value: { history: pointHistory },
          metadata: {},
        },
      },
      fiwareService,
      fiwareServicePath,
    );
  }
  getUpdatedPointHistory(
    pointHistory: PointHistory,
    currentPoint: Point,
  ): PointHistory {
    const updatedPointHistory = pointHistory;
    updatedPointHistory.push(
      {
        time: new Date(),
        latitude: currentPoint.latitude,
        longitude: currentPoint.longitude,
      },
    );
    if (updatedPointHistory.length >= this.allowableCount) {
      return updatedPointHistory.slice(
        updatedPointHistory.length - this.allowableCount,
        updatedPointHistory.length,
      );
    }

    return updatedPointHistory;
  }
  async isStuck(
    point: Point,
    entityId: string,
    fiwareService: string,
    fiwareServicePath: string,
  ): Promise<boolean> {
    const entity = await this.orionClient.getEntity<NGSIStuckCheckerEntity>(
      `${POINT_HISTORY_ID_PREFIX}_${entityId}`,
      fiwareService,
      fiwareServicePath,
    );
    if (entity.pointHistory.value.history.length < this.allowableCount) {
      return false;
    }
    return entity.pointHistory.value.history.every((p) =>
      this.range >
        this.calculateDistance(
          point,
          { latitude: p.latitude, longitude: p.longitude },
        )
    );
  }

  // Calculate the earth as a true sphere.
  // Correctly, it should be calculated considering GRS80
  calculateDistance(point1: Point, point2: Point) {
    const EARTH_RADIUS = 6371000;

    const distanceMeters = EARTH_RADIUS * Math.acos(
      Math.cos(point1.latitude / 180 * Math.PI) *
          Math.cos((point2.longitude - point1.longitude) / 180 * Math.PI) *
          Math.cos(point2.latitude / 180 * Math.PI) +
        Math.sin(point1.latitude / 180 * Math.PI) *
          Math.sin(point2.latitude / 180 * Math.PI),
    );
    return distanceMeters;
  }
}
