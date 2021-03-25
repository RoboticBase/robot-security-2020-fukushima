import { assertEquals, Stub, stub } from "../deps.ts";
import { OrionClient } from "../src/orion_client.ts";
import { DEFAULT_ORION_ENDPOINT } from "../src/const.ts";

const orionClient = new OrionClient();
const ID = "test01";
const ATTR_DATA = {};
const TYPE = "test";
const FIWARE_SERVICE = "fiwareservice";
const FIWARE_SERVICE_PATH = "fiwareservicepath";

Deno.test("Test fiware orion client", async () => {
  const request: Stub<OrionClient> = stub(
    orionClient,
    "request",
    ["success"],
  );
  assertEquals(
    await orionClient.patchAttr<Record<string, unknown>>(
      ID,
      ATTR_DATA,
      TYPE,
      FIWARE_SERVICE,
      FIWARE_SERVICE_PATH,
    ),
    "success",
  );

  const url = `${DEFAULT_ORION_ENDPOINT}/v2/entities/${ID}/attrs?type=${TYPE}`;
  assertEquals(
    request.calls,
    [
      {
        args: [
          url,
          "PATCH",
          JSON.stringify(ATTR_DATA),
          FIWARE_SERVICE,
          FIWARE_SERVICE_PATH,
        ],
        self: orionClient,
        returned: "success",
      },
    ],
  );
});
