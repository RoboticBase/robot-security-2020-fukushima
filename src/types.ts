export interface Point {
  latitude: number;
  longitude: number;
}

export type BlockPoints = [Point, Point];

export type NGSIRobotPose = {
  data: [
    {
      id: string;
      type: string;
      pose: {
        type: string;
        value: {
          angle: { theta: number };
          point: { latitude: number; longitude: number; altitude: number };
        };
        metadata: {};
      };
    },
  ];
};

export type NGSIAlertCommand = {
  alertCmd: {
    value: string
  }
}