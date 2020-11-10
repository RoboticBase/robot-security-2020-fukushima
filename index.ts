import { Application, log, Router, Status } from "./deps.ts";
import { GeoFence } from "./src/geo_fence.ts";
import type { NGSIAlertCommand, NGSIRobotPose } from "./src/types.ts";
import { OrionClient } from "./src/orion_client.ts";
import {
  DEFAULT_ALERT_DATA_NAME,
  DEFAULT_ENTITY_ID,
  DEFAULT_ENTITY_TYPE,
  DEFAULT_FIWARE_SERVICE,
  DEFAULT_FIWARE_SERVICEPATH,
  DEFAULT_ORION_ENDPOINT,
} from "./src/const.ts";

const logger = log.getLogger();

const router = new Router();
const geoDataPath = Deno.env.get("GEO_DATA_PATH") || "./data/geo_fence.json";
const geoFence = new GeoFence(geoDataPath);
const orionClient = new OrionClient(DEFAULT_ORION_ENDPOINT);
router
  .post("/", async (context) => {
    const body = context.request.body({ type: "json" });
    try {
      const pose: NGSIRobotPose = await body.value;
      const latitude = pose.data[0].pose.value.point.latitude;
      const longitude = pose.data[0].pose.value.point.longitude;
      const robotId = pose.data[0].id;
      const result = geoFence.isRange({ latitude, longitude });
      context.response.body = {
        "result": result,
      };
      if (result) {
        logger.info(
          {
            "message": `Security Geo Fence: OK, "${robotId}" is range of fence`,
            latitude: latitude,
            longitude: longitude,
          },
        );
      } else {
        const response = await orionClient.patchAttr<NGSIAlertCommand>(
          DEFAULT_ENTITY_ID,
          { alertCmd: { value: DEFAULT_ALERT_DATA_NAME } },
          DEFAULT_ENTITY_TYPE,
          DEFAULT_FIWARE_SERVICE,
          DEFAULT_FIWARE_SERVICEPATH,
        );
        logger.debug(response);
        logger.error(
          {
            "message":
              `Security Geo Fence: Error, "${robotId}" is out of fence`,
            latitude: latitude,
            longitude: longitude,
          },
        );
      }
    } catch (e) {
      context.response.status = Status.BadRequest;
      context.response.body = { "error": e.message };
      log.error({ "message": "Security Geo Fence: Bad Request" });
    }
  });

const app = new Application();
app.use(router.routes());
app.use(router.allowedMethods());

await app.listen({ port: 8000 });
