export const DEFAULT_ORION_ENDPOINT = Deno.env.get("ORION_ENDPOINT") ||
"http://orion:1026";
export const DEFAULT_ENTITY_ID = Deno.env.get("ENTITY_ID") || "robot01";
export const DEFAULT_ENTITY_TYPE = Deno.env.get("ENTITY_TYPE") || "robot";
export const DEFAULT_FIWARE_SERVICE = Deno.env.get("FIWARE_SERVICE") ||
"security";
export const DEFAULT_FIWARE_SERVICEPATH = Deno.env.get("FIWARE_SERVICEPATH") ||
"/";
export const DEFAULT_ALERT_DATA_NAME =  Deno.env.get("FIWARE_SERVICEPATH") ||
"geo_fence_alert";