import { DEFAULT_ORION_ENDPOINT } from "./const.ts";

class OrionClient {
  constructor(private endpoint: string = DEFAULT_ORION_ENDPOINT) {
  }
  async getEntity<T>(
    id: string,
    fiwareService: string,
    fiwareServicePath: string,
  ): Promise<T>{
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
}
