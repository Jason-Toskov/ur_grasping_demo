from pydantic import BaseModel


class TrajectoryModel(BaseModel):
    scene_scan: str


class PathModel(BaseModel):
    root: str
    key_locs: str
    trajectories: TrajectoryModel


class Config(BaseModel):
    paths: PathModel
