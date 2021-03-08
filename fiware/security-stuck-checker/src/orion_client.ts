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
    const response = await this.getRequest(
      url,
      fiwareService,
      fiwareServicePath,
    );
    return await response.json();
  }

  async getAttrs<T>(
    id: string,
    attrs: string,
    fiwareService: string,
    fiwareServicePath: string,
  ): Promise<T> {
    const url = `${this.endpoint}/v2/entities/${id}/attrs/${attrs}`;
    const response = await this.getRequest(
      url,
      fiwareService,
      fiwareServicePath,
    );
    return await response.json();
  }

  async patchAttr<T>(
    id: string,
    attrData: T,
    type: string,
    fiwareService: string,
    fiwareServicePath: string,
  ): Promise<Response> {
    const url = `${this.endpoint}/v2/entities/${id}/attrs?type=${type}`;
    const response = await this.patchRequest(
      url,
      JSON.stringify(attrData),
      fiwareService,
      fiwareServicePath,
    );
    return response;
  }
  async getRequest(
    url: string,
    fiwareService: string,
    fiwareServicePath: string,
  ) {
    const response = await fetch(
      url,
      {
        headers: {
          "Content-type": "application/json; charset=UTF-8",
          "fiware-service": fiwareService,
          "fiware-servicepath": fiwareServicePath,
        },
      },
    );
    return response;
  }
  async patchRequest(
    url: string,
    body: string,
    fiwareService: string,
    fiwareServicePath: string,
  ) {
    const response = await fetch(
      url,
      {
        method: "PATCH",
        body: body,
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
