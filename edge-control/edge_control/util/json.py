import json
from typing import Any, Type, TypeVar

import dataclasses
from dacite import from_dict

T = TypeVar("T")


class EncodeDataclasses(json.JSONEncoder):
    def default(self, o: Any):
        if dataclasses.is_dataclass(o):
            return dataclasses.asdict(o)
        return super().default(o)


def dumps(o: Any) -> str:
    # TODO: remove key: None entries
    return json.dumps(o, cls=EncodeDataclasses)


def loads(data: Any, data_class: Type[T]) -> T:
    return from_dict(data_class=data_class, data=json.loads(data))
