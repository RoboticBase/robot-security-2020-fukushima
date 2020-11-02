import { getEnvNumberValue } from "./utils.ts";

export const DEFAULT_ALLOWABLE_COUNT = getEnvNumberValue("ALLOWABLE_COUNT", 5);
export const DEFAULT_RANGE_METERS = getEnvNumberValue("RANGE_METERS", 1);
export const DEFAULT_LOOP_SLEEP_SECOND = 5;
export const DEFAULT_ORION_ENDPOINT = Deno.env.get("ORION_ENDPOINT") ||
  "http://orion:1026";
export const DEFAULT_ENTITY_ID = Deno.env.get("ENTITY_ID") || "robot01";
export const DEFAULT_FIWARE_SERVICE = Deno.env.get("FIWARE_SERVICE") ||
  "security";
export const DEFAULT_FIWARE_SERVICEPATH = Deno.env.get("FIWARE_SERVICEPATH") ||
  "/";
export const POINT_HISTORY_ID_PREFIX =
  Deno.env.get("POINT_HISTORY_ID_PREFIX") || "point_history";
