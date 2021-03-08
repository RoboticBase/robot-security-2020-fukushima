export function sleep(seconds: number) {
  return new Promise((resolve) => setTimeout(resolve, seconds * 1000));
}
export function getEnvNumberValue(key: string, defaultValue: number): number {
    if (Deno.env.get(key)) {
      const param = Deno.env.get(key);
      if (param !== undefined) {
        const parsedValue = parseInt(param);
        if (isNaN(parsedValue)) {
          return defaultValue;
        }
        return parsedValue;
      }
    }
    return defaultValue;
}