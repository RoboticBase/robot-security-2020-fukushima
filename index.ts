import { Application, Router } from "https://deno.land/x/oak@v6.3.1/mod.ts";
import { GeoFence } from "./src/geo_fence.ts";

const router = new Router();
const geoFence = new GeoFence("./data/geo_blocks.json");
router
  .post("/", (context) => {
    context.response.body = "";
  });

const app = new Application();
app.use(router.routes());
app.use(router.allowedMethods());

await app.listen({ port: 8000 });