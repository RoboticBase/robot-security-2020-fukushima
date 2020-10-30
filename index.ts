import { Application, Router } from "https://deno.land/x/oak@v6.3.1/mod.ts";
import { GeoFence } from "./src/geo_fence.ts";
import type { Point } from "./src/point.ts";

const router = new Router();
const geoFence = new GeoFence("./data/geo_blocks.json");
router
  .post("/", async (context) => {
    const body = context.request.body({ type: "json" });
    const targetPoint: Point = await body.value;
    context.response.body = { "result": geoFence.isRange(targetPoint) };
  });

const app = new Application();
app.use(router.routes());
app.use(router.allowedMethods());

await app.listen({ port: 8000 });
