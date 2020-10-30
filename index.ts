import { Application, log, Router, Status } from "./deps.ts";
import { GeoFence } from "./src/geo_fence.ts";
import type { NGSIRobotPose } from "./src/types.ts";

const logger = log.getLogger();

const router = new Router();
const geoDataPath = Deno.env.get("GEO_DATA_PATH") || "./data/geo_fence.json";
const geoFence = new GeoFence(geoDataPath);
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
        log.info(
          {
            "message": `Security Geo Fence: OK, "${robotId}" is range of fence`,
            latitude: latitude,
            longitude: longitude,
          },
        );
      } else {
        log.error(
          {
            "message": `Security Geo Fence: Error, "${robotId}" is out of fence`,
            latitude: latitude,
            longitude: longitude,
          },
        );
      }
    } catch (e) {
      context.response.status = Status.BadRequest;
      context.response.body = { "error": e.message };
    }
  });

const app = new Application();
app.use(router.routes());
app.use(router.allowedMethods());

await app.listen({ port: 8000 });
