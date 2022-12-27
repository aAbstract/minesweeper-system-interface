from typing import Optional, Any
from pydantic import BaseModel


class service_response(BaseModel):
    success: Optional[bool] = True
    msg: Optional[str] = ''
    data: Optional[Any] = None
