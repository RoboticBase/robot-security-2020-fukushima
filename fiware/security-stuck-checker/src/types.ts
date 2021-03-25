export interface Point {
  latitude: number;
  longitude: number;
}

export type PointTime = { time: Date; latitude: number; longitude: number };
export type PointHistory = Array<PointTime>;

export type NGSIStuckCheckerEntity = {
  id: string;
  type: string;
  pointHistory: {
    type: string;
    value: { history: PointHistory };
    metadata: {};
  };
  metadata: {};
};
export type NGSIPointHistoryAttribute = {
  pointHistory: {
    type: string;
    value: { history: PointHistory };
    metadata: {};
  };
};

export type NGSIPoseAttribute = {
  type: string;
  value: {
    angle: {
      theta: number;
    };
    point: {
      altitude: number;
      latitude: number;
      longitude: number;
    };
  };
  metadata: {};
};

export type NGSIModeAttribute = {
  type: string;
  value: string;
  metadata: {};
};

export type NGSIAlertCommand = {
  alertCmd: {
    value: string;
  };
};
