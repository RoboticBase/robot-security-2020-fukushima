export class Parameters {
  static test<T>(
    name: string,
    parameters: Array<T>,
    testFunction: (param: T) => void | Promise<void>,
  ) {
    parameters.forEach((parameter: T, index: number) => {
      Deno.test(`${name}_${index}`, () => {
        return testFunction(parameter);
      });
    });
  }
}
