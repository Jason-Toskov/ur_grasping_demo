import os

from pydantic import BaseModel, Field

# Get loc of package on computer
ROOT_PATH = os.path.dirname(__file__)


class TrajectoryModel(BaseModel):
    scene_scan: str
    single_view: str


class PathModel(BaseModel):
    # Statically add the root location to the pydantic model
    root: str = Field(ROOT_PATH, const=True)
    key_locs: str
    trajectories: TrajectoryModel


class Config(BaseModel):
    paths: PathModel
