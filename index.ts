import {
  DEFAULT_ENTITY_ID,
  DEFAULT_FIWARE_SERVICE,
  DEFAULT_FIWARE_SERVICEPATH,
} from "./src/const.ts";
import { StuckChecker } from "./src/stuck_checker.ts";

await new StuckChecker().startLoop(
  DEFAULT_ENTITY_ID,
  DEFAULT_FIWARE_SERVICE,
  DEFAULT_FIWARE_SERVICEPATH,
);
