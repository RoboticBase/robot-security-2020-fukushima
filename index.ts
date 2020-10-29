import { Application, Router } from "https://deno.land/x/oak@v6.3.1/mod.ts";

const router = new Router();
router
  .post("/", (context) => {
    context.response.body = "";
  });

const app = new Application();
app.use(router.routes());
app.use(router.allowedMethods());

await app.listen({ port: 8000 });