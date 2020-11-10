import { DEFAULT_ORION_ENDPOINT } from "./const.ts";

export class OrionClient {
  constructor(private endpoint: string = DEFAULT_ORION_ENDPOINT) {
  }
  async patchAttr<T>(
    id: string,
    attrData: T,
    type: string,
    fiwareService: string,
    fiwareServicePath: string,
  ) {
    const url = `${this.endpoint}/v2/entities/${id}/attrs?type=${type}`;
    const response = await fetch(
      url,
      {
        method: "PATCH",
        body: JSON.stringify(attrData),
        headers: {
          "Content-type": "application/json; charset=UTF-8",
          "fiware-service": fiwareService,
          "fiware-servicepath": fiwareServicePath,
        },
      },
    );
    return response;
  }
}
