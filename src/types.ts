export interface Point {
  latitude: number;
  longitude: number;
}

export type PointTime = { time: Date; latitude: number; longitude: number };
export type PointHistory = [PointTime];

export type NGSIPointHistoryEntity = {
  id: string;
  type: string;
  pointHistory: {
    type: string;
    value: { history: PointHistory };
    metadata: {};
  };
  metadata: {};
};

export type NGSIPoseAttribute = {
  type: string;
  value: {
    angle: {
      theta: number;
    };
    point: {
      altitude: number;
      latitdue: number;
      longitude: number;
    };
  };
  metadata: {};
};
