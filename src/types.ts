export interface Point {
  latitude: number;
  longitude: number;
}

export type PointTime = { time: Date; latitude: number; longitude: number };
export type pointHistory = [PointTime]

export type NGSIPointHistoryEntity = {
  id: string;
  type: string;
  value: {
    pointHistory: pointHistory
  };
  metadata: {};
};
