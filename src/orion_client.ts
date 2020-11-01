import { DEFAULT_ORION_ENDPOINT } from "./const.ts";

export class OrionClient {
  constructor(private endpoint: string = DEFAULT_ORION_ENDPOINT) {
  }
  async getEntity<T>(
    id: string,
    fiwareService: string,
    fiwareServicePath: string,
  ): Promise<T> {
    const url = `${this.endpoint}/v2/entities/${id}`;
    const response = await fetch(
      url,
      {
        headers: {
          "fiware-service": fiwareService,
          "fiware-servicepath": fiwareServicePath,
        },
      },
    );
    return await response.json();
  }

  async getAttrs<T>(
    id: string,
    attrs: string,
    fiwareService: string,
    fiwareServicePath: string,
  ): Promise<T> {
    const url = `${this.endpoint}/v2/entities/${id}?attrs=${attrs}`;
    const response = await fetch(
      url,
      {
        headers: {
          "fiware-service": fiwareService,
          "fiware-servicepath": fiwareServicePath,
        },
      },
    );
    return await response.json();
  }

  async patchAttr<T>(
    id: string,
    attrData: T,
    fiwareService: string,
    fiwareServicePath: string,
  ) {
    const url = `${this.endpoint}/v2/entities/${id}/attrs`;
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
    return await response.json();
  }
}
