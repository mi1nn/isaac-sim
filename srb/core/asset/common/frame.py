from pydantic import BaseModel

from srb.core.asset.common.transform import Transform


class Frame(BaseModel):
    prim_relpath: str = ""
    offset: Transform = Transform()
